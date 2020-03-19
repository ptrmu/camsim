

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"

namespace camsim
{
  template<typename TCalibrationModel>
  class SolverOpencv : public SolverInterface<TCalibrationModel>
  {
    const SolverRunnerBase<TCalibrationModel> &sr_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". Not quite sure why the RVO doesn't prevent this but it might
      // be because the object is passed by the operator() member and not by the
      // class itself.
      if (graph_.size() < 2) {
        return;
      }
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
      auto camera_key{fd.camera_.key_};

      for (auto &board_data : fd.board_datas_) {
        auto board_key{board_data.board_.key_};

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
