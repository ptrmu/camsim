

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"

namespace camsim
{
  template<typename TCalibrationModel>
  class SolverOpencv
  {
    SolverRunner<TCalibrationModel> &sr_;
    const bool auto_initial_;

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
    explicit SolverOpencv(SolverRunner<TCalibrationModel> &sr, bool auto_initial) :
      sr_{sr}, auto_initial_{auto_initial}
    {
      sr_.add_marker_0_prior(graph_, initial_);
    }

    ~SolverOpencv()
    {
      display_results();
    }

    void operator()(const FrameData<TCalibrationModel> &fd)
    {
      auto camera_key{fd.camera_.key_};

      // Add the initial camera pose estimate
      if (!auto_initial_) {
        initial_.insert(camera_key, fd.camera_f_world_perturbed_);
      }

      for (auto &marker_data : fd.marker_datas_) {
        auto marker_key{marker_data.marker_.key_};

      }
    }
  };


  template<typename TCalibrationModel>
  std::function<void(FrameData<TCalibrationModel> &)>
  solver_opencv_factory(SolverRunner<TCalibrationModel> &sr)
  {
    return SolverOpencv<TCalibrationModel>(sr);
  }

  int cal_solver_opencv_checkerboard()
  {

    return EXIT_SUCCESS;
  }
}
