

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/calib3d.hpp"

namespace camsim
{
  template<typename TCalibrationModel>
  class SolverOpencv : public SolverInterface<TCalibrationModel>
  {
    const SolverRunnerBase<TCalibrationModel> &sr_;

    std::vector<std::vector<cv::Vec3f>> objectPoints_{};
    std::vector<std::vector<cv::Vec2f>> imagePoints_{};

    void display_results()
    {
      cv::Mat cameraMatrix;
      cv::Mat distCoeffs;
      cv::Mat rvecs;
      cv::Mat tvecs;
      cv::Mat newObjPoints;
      cv::Mat stdDeviationsIntrinsics;
      cv::Mat stdDeviationsExtrinsics;
      cv::Mat stdDeviationsObjPoints;
      cv::Mat perViewErrors;

      auto err = calibrateCamera(objectPoints_,
                                         imagePoints_,
                                         cv::Size{800, 600},
                                         cameraMatrix,
                                         distCoeffs,
                                         rvecs,
                                         tvecs,
                                         stdDeviationsIntrinsics,
                                         stdDeviationsExtrinsics,
                                         perViewErrors);

      std::cout << cameraMatrix << std::endl << distCoeffs << std::endl;
      int t = 6;
    }

  public:
    explicit SolverOpencv(const SolverRunnerBase<TCalibrationModel> &sr) :
      sr_{sr}
    {}

    ~SolverOpencv() override
    {
      display_results();
    }

    void add_frame(const FrameData<TCalibrationModel> &fd) override
    {
      for (auto &board_data : fd.board_datas_) {
        std::vector<cv::Vec3f> cv_junctions_f_board;
        std::vector<cv::Vec2f> cv_junctions_f_image;

        for (int i = 0; i < sr_.model_.boards_.junctions_f_board_.size(); i += 1) {

          auto &junction_f_board = sr_.model_.boards_.junctions_f_board_[i];
          cv_junctions_f_board.emplace_back(cv::Vec3f(junction_f_board(0),
                                                      junction_f_board(1),
                                                      junction_f_board(2)));

          auto &junction_f_image = board_data.junctions_f_image_perturbed_[i].junction_;
          cv_junctions_f_image.emplace_back(cv::Vec2f(junction_f_image(0),
                                                      junction_f_image(1)));
        }

        objectPoints_.emplace_back(cv_junctions_f_board);
        imagePoints_.emplace_back(cv_junctions_f_image);
      }
    }
  };


  template<typename TCalibrationModel>
  struct SolverOpencvFactoryImpl : public SolverFactoryInterface<TCalibrationModel>
  {
    std::unique_ptr<SolverInterface<TCalibrationModel>> new_solver(
      const SolverRunnerBase<TCalibrationModel> &solver_runner) override
    {
      return std::make_unique<SolverOpencv<TCalibrationModel>>(solver_runner);
    }
  };

  template<>
  std::unique_ptr<SolverFactoryInterface<CheckerboardCalibrationModel>>
  solver_opencv_factory<CheckerboardCalibrationModel>()
  {
    return std::make_unique<SolverOpencvFactoryImpl<CheckerboardCalibrationModel>>();
  }

  template<>
  std::unique_ptr<SolverFactoryInterface<CharucoboardCalibrationModel>>
  solver_opencv_factory<CharucoboardCalibrationModel>()
  {
    return std::make_unique<SolverOpencvFactoryImpl<CharucoboardCalibrationModel>>();
  }
}
