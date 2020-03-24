

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <opencv2/opencv.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

namespace camsim
{

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectCalibrateFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Cal3DS2>
  {
    const gtsam::Point3 junction_f_board_;
    const gtsam::Point2 junction_f_image_;

  public:
    ProjectCalibrateFactor(gtsam::Point2 junction_f_image,
                           const gtsam::SharedNoiseModel &model,
                           gtsam::Key camera_key,
                           gtsam::Key calibration_key,
                           gtsam::Point3 junction_f_board) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Cal3DS2>(model, camera_key, calibration_key),
      junction_f_board_(std::move(junction_f_board)),
      junction_f_image_(std::move(junction_f_image))
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &camera_f_board,
                                const gtsam::Cal3DS2 &cal3ds2,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
      gtsam::Matrix26 d_point2_wrt_pose3;
      gtsam::Matrix29 d_point2_wrt_cal;

      // Project this point to the camera's image frame. Catch and return a large error
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_board, cal3ds2};
      try {
        gtsam::Point2 point_f_image = camera.project(
          junction_f_board_,
          H1 ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none,
          boost::none,
          H2 ? gtsam::OptionalJacobian<2, 9>(d_point2_wrt_cal) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_cal;
        }

        // Return the error.
        return point_f_image - junction_f_image_;

      } catch (gtsam::CheiralityException &e) {
      }
      if (H1) *H1 = gtsam::Matrix::Zero(2, 6);
      if (H2) *H2 = gtsam::Matrix::Zero(2, 9);
      return gtsam::Vector2{2.0 * cal3ds2.px(), 2.0 * cal3ds2.py()};
    }
  };

  template<typename TModel>
  class SolverProjectBetween : public SolverInterface<TModel>
  {
    const SolverRunnerBase<TModel> &sr_;

    std::vector<std::vector<cv::Vec3f>> objectPoints_{};
    std::vector<std::vector<cv::Vec2f>> imagePoints_{};

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};

    std::uint64_t calibration_key_{gtsam::Symbol('K', 0).key()};

  public:
    explicit SolverProjectBetween(const SolverRunnerBase<TModel> &sr) :
      sr_{sr}
    {}

    void add_frame(const FrameData<TModel> &fd) override
    {
      auto camera_key{fd.camera_.key_};

      for (auto &board_data : fd.board_datas_) {

        std::vector<cv::Vec3f> cv_junctions_f_board;
        std::vector<cv::Vec2f> cv_junctions_f_image;

        for (int i = 0; i < sr_.model_.boards_.junctions_f_board_.size(); i += 1) {

          // set up arguments for opencv calibrateCamera.
          auto &junction_f_board = sr_.model_.boards_.junctions_f_board_[i];
          cv_junctions_f_board.emplace_back(cv::Vec3f(junction_f_board(0),
                                                      junction_f_board(1),
                                                      junction_f_board(2)));

          auto &junction_f_image = board_data.junctions_f_image_perturbed_[i].junction_;
          cv_junctions_f_image.emplace_back(cv::Vec2f(junction_f_image(0),
                                                      junction_f_image(1)));

          // Set up the graph for the gtsam optimization
          graph_.emplace_shared<ProjectCalibrateFactor>(junction_f_image,
                                                        sr_.point2_noise_,
                                                        camera_key,
                                                        calibration_key_,
                                                        junction_f_board);
        }

        objectPoints_.emplace_back(cv_junctions_f_board);
        imagePoints_.emplace_back(cv_junctions_f_image);

        auto camera_f_board = board_data.board_.board_f_world_.inverse() * fd.camera_.camera_f_world_;
        initial_.insert(camera_key, camera_f_board);
      }
    }

    void calc_opencv()
    {
      cv::Mat cameraMatrix;
      cv::Mat distCoeffs;
      cv::Mat rvecs;
      cv::Mat tvecs;

      auto err = calibrateCamera(objectPoints_, imagePoints_,
                                 cv::Size{800, 600},
                                 cameraMatrix, distCoeffs,
                                 rvecs, tvecs,
                                 0);

      gtsam::Cal3DS2{
        cameraMatrix.at<double>(0, 0),  // fx
        cameraMatrix.at<double>(1, 1),  // fy
        cameraMatrix.at<double>(0, 1), // s
        cameraMatrix.at<double>(0, 2),  // u0
        cameraMatrix.at<double>(1, 2),  // v0
        distCoeffs.at<double>(0), // k1
        distCoeffs.at<double>(1), // k2
        distCoeffs.at<double>(2), // p1
        distCoeffs.at<double>(3)}// p2
      .print("opencv");
    }

    void set_initial_model()
    {
      for (auto &board : sr_.model.boards_boards_) {

//        initial_.insert(board.key_, )
      }
    }

    typename TModel::Result solve() override
    {
      calc_opencv();

      initial_.insert(calibration_key_, sr_.model_.cameras_.calibration_);

      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setMaxIterations(1000);
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);


      std::cout << std::endl << "Solve with perfect initial estimates" << std::endl;
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;


      auto cal3ds2 = result.at<gtsam::Cal3DS2>(calibration_key_);
      cal3ds2.print("cal3ds2\n");

//      auto marginals_slam_ptr = sr_.get_marginals(graph_, result);
//      for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
//        sr_.display_results(PoseWithCovariance::Extract(result, marginals_slam_ptr.get(), m.key));
//      }

      return typename TModel::Result{gtsam::Cal3DS2{}};
    }
  };


  template<typename TModel>
  struct SolverGtsamProjectBetweenFactoryImpl : public SolverFactoryInterface<TModel>
  {
    std::unique_ptr<SolverInterface<TModel>> new_solver(
      const SolverRunnerBase<TModel> &solver_runner) override
    {
      return std::make_unique<SolverProjectBetween<TModel>>(solver_runner);
    }
  };

  template<>
  std::unique_ptr<SolverFactoryInterface<CheckerboardCalibrationModel>>
  solver_project_between_factory<CheckerboardCalibrationModel>()
  {
    return std::make_unique<SolverGtsamProjectBetweenFactoryImpl<CheckerboardCalibrationModel>>();
  }

  template<>
  std::unique_ptr<SolverFactoryInterface<CharucoboardCalibrationModel>>
  solver_project_between_factory<CharucoboardCalibrationModel>()
  {
    return std::make_unique<SolverGtsamProjectBetweenFactoryImpl<CharucoboardCalibrationModel>>();
  }
}
