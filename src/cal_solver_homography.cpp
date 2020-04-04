

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

namespace camsim
{
  template<typename TModel>
  class SolverHomography : public SolverInterface<TModel>
  {
    const SolverRunnerBase<TModel> &sr_;

  public:
    explicit SolverHomography(const SolverRunnerBase<TModel> &sr) :
      sr_{sr}
    {}

    void add_frame(const FrameData<TModel> &fd) override
    {
      for (auto &board_data : fd.board_datas_) {
        std::vector<cv::Vec2f> cv_junctions_f_board;
        std::vector<cv::Vec2f> cv_junctions_f_image;

        for (int i = 0; i < sr_.model_.boards_.junctions_f_board_.size(); i += 1) {

          auto &junction_f_board = sr_.model_.boards_.junctions_f_board_[i];
          cv_junctions_f_board.emplace_back(cv::Vec2f(junction_f_board(0),
                                                      junction_f_board(1)));

          auto &junction_f_image = board_data.junctions_f_image_perturbed_[i].junction_;
          cv_junctions_f_image.emplace_back(cv::Vec2f(junction_f_image(0),
                                                      junction_f_image(1)));
        }

        gtsam::Pose3 board_f_camera = fd.camera_.camera_f_world_.inverse() * board_data.board_.board_f_world_;

        gtsam::Matrix3 rt = board_f_camera.rotation().matrix();
        rt(0, 2) = board_f_camera.translation().x();
        rt(1, 2) = board_f_camera.translation().y();
        rt(2, 2) = board_f_camera.translation().z();
        gtsam::Cal3DS2 const &cal = sr_.model_.cameras_.calibration_;
        gtsam::Matrix3 m = (gtsam::Matrix3{} << cal.fx(), 0., cal.px(), 0., cal.fy(), cal.py(), 0., 0., 1.).finished();
        auto mrt = m * rt;
        std::cout << mrt << std::endl;

        auto homo = cv::findHomography(cv_junctions_f_board, cv_junctions_f_image);
        std::cout << homo << std::endl;

        std::cout << homo * mrt(0, 0) / homo.at<double>(0, 0) << std::endl;
        std::cout << std::endl;
      }
    }

    typename TModel::Result solve() override
    {
      return typename TModel::Result{gtsam::Cal3DS2{}};
    }
  };


  template<typename TModel>
  struct SolverHomographyFactoryImpl : public SolverFactoryInterface<TModel>
  {
    std::unique_ptr<SolverInterface<TModel>> new_solver(
      const SolverRunnerBase<TModel> &solver_runner) override
    {
      return std::make_unique<SolverHomography<TModel>>(solver_runner);
    }
  };

  template<>
  std::unique_ptr<SolverFactoryInterface<CheckerboardCalibrationModel>>
  solver_homography_factory<CheckerboardCalibrationModel>()
  {
    return std::make_unique<SolverHomographyFactoryImpl<CheckerboardCalibrationModel>>();
  }

  template<>
  std::unique_ptr<SolverFactoryInterface<CharucoboardCalibrationModel>>
  solver_homography_factory<CharucoboardCalibrationModel>()
  {
    return std::make_unique<SolverHomographyFactoryImpl<CharucoboardCalibrationModel>>();
  }
}
