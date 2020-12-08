
#ifndef _CAL_SOLVER_RUNNER_HPP
#define _CAL_SOLVER_RUNNER_HPP

#include "calibration_model.hpp"
#include "pose_with_covariance.hpp"

#define ENABLE_TIMING

#include <gtsam/base/timing.h>
#include <gtsam/linear/NoiseModel.h>
#include "gtsam/linear/Sampler.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

namespace camsim
{

// ==============================================================================
// SolverRunnerBase
// ==============================================================================

  template<typename TModel>
  struct SolverRunnerBase
  {
    const TModel &model_;
    const gtsam::Vector6 pose3_sampler_sigmas_;
    const gtsam::Vector2 point2_sampler_sigmas_;
    const bool print_covariance_;

    const gtsam::SharedNoiseModel pose3_noise_;
    const gtsam::SharedNoiseModel point2_noise_;
    const gtsam::SharedNoiseModel point2x4_noise_;

    gtsam::Sampler pose3_sampler_;
    gtsam::Sampler point2_sampler_;
    int frames_processed_{0};

    SolverRunnerBase(const TModel &model,
                     const gtsam::Vector6 &pose3_sampler_sigmas,
                     const gtsam::Vector6 &pose3_noise_sigmas,
                     const gtsam::Vector2 &point2_sampler_sigmas,
                     const gtsam::Vector2 &point2_noise_sigmas,
                     bool print_covariance) :
      model_{model},
      pose3_sampler_sigmas_{pose3_sampler_sigmas},
      point2_sampler_sigmas_{point2_sampler_sigmas},
      print_covariance_{print_covariance},
      pose3_noise_{gtsam::noiseModel::Diagonal::Sigmas(pose3_noise_sigmas)},
      point2_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas)},
      point2x4_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas.replicate<4, 1>())},
      pose3_sampler_(pose3_sampler_sigmas),
      point2_sampler_(point2_sampler_sigmas)
    {}

    gtsam::Pose3 get_perturbed_camera_f_world(const CameraModel &camera)
    {
      return camera.camera_f_world_.retract(pose3_sampler_.sample());
    }

    std::vector<JunctionFImage> get_perturbed_junctions_f_image(const std::vector<JunctionFImage> &junctions_f_image)
    {
      std::vector<JunctionFImage> junctions_f_image_perturbed{};

      for (auto &junction_f_image : junctions_f_image) {
        auto p{junction_f_image.junction_};
        p += gtsam::Point2(point2_sampler_.sample());
        junctions_f_image_perturbed.emplace_back(JunctionFImage(
            junction_f_image.visible_,
            junction_f_image.camera_key_,
            junction_f_image.board_key_,
            junction_f_image.junction_id_,
            p));
      }

      return junctions_f_image_perturbed;
    }

    std::unique_ptr<gtsam::Marginals> get_marginals(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values) const
    {
      try {
        return std::make_unique<gtsam::Marginals>(graph, values, gtsam::Marginals::QR);
      }
      catch (gtsam::IndeterminantLinearSystemException &ex) {
      }
      std::cout << "Could not create marginals, trying QR" << std::endl;

      try {
        return std::make_unique<gtsam::Marginals>(graph, values, gtsam::Marginals::QR);
      }
      catch (gtsam::IndeterminantLinearSystemException &ex) {
      }
      std::cout << "Could not create marginals, returning null" << std::endl;

      return std::unique_ptr<gtsam::Marginals>{};
    }
  };


// ==============================================================================
// BoardData
// ==============================================================================

  template<typename TModel>
  struct BoardData
  {
    const typename TModel::TargetModel &board_;
    const std::vector<JunctionFImage> junctions_f_image_perturbed_;

    BoardData(const typename TModel::TargetModel &board,
              const std::vector<JunctionFImage> &junctions_f_image_perturbed) :
      board_{board},
      junctions_f_image_perturbed_{junctions_f_image_perturbed}
    {}

    static void load_board_datas(SolverRunnerBase<TModel> sr,
                                 const CameraModel &camera,
                                 const typename TModel::TargetModel &board,
                                 std::vector<BoardData<TModel>> &board_datas);
  };

  using CheckerboardData = BoardData<CheckerboardCalibrationModel>;

  struct CharucoboardData : public BoardData<CharucoboardCalibrationModel>
  {
    std::vector<ArucoCornersFImage> arucos_corners_f_image_perturbed_;

    CharucoboardData(const typename CharucoboardCalibrationModel::TargetModel &board,
                     const std::vector<JunctionFImage> &junctions_f_image_perturbed,
                     const std::vector<ArucoCornersFImage> &arucos_corners_f_image_perturbed) :
      BoardData<CharucoboardCalibrationModel>(board, junctions_f_image_perturbed),
      arucos_corners_f_image_perturbed_{arucos_corners_f_image_perturbed}
    {}
  };

// ==============================================================================
// FrameData
// ==============================================================================

  template<typename TModel>
  struct FrameData
  {
    const CameraModel &camera_;
    const gtsam::Pose3 camera_f_world_perturbed_;
    const std::vector<BoardData<TModel>> board_datas_;

    FrameData(const CameraModel &camera,
              gtsam::Pose3 camera_f_world_perturbed,
              std::vector<BoardData<TModel>> board_datas) :
      camera_{camera},
      camera_f_world_perturbed_{std::move(camera_f_world_perturbed)},
      board_datas_{std::move(board_datas)}
    {}
  };

// ==============================================================================
// SolverInterface
// ==============================================================================

  template<typename TModel>
  struct SolverInterface
  {
    virtual ~SolverInterface() = default; //
    virtual void add_frame(const FrameData<TModel> &frame_data) = 0; //
    virtual typename TModel::Result solve() = 0; //
  };

  template<typename TModel>
  struct SolverFactoryInterface
  {
    virtual ~SolverFactoryInterface() = default; //
    virtual std::unique_ptr<SolverInterface<TModel>> new_solver(
      const SolverRunnerBase<TModel> &solver_runner) = 0; //
  };

// ==============================================================================
// SolverRunner
// ==============================================================================

  template<typename TModel>
  struct SolverRunner : public SolverRunnerBase<TModel>
  {
    SolverRunner(const TModel &model,
                 const gtsam::Vector6 &pose3_sampler_sigmas,
                 const gtsam::Vector6 &pose3_noise_sigmas,
                 const gtsam::Vector2 &point2_sampler_sigmas,
                 const gtsam::Vector2 &point2_noise_sigmas,
                 bool print_covariance) :
      SolverRunnerBase<TModel>{model,
                               pose3_sampler_sigmas, pose3_noise_sigmas,
                               point2_sampler_sigmas, point2_noise_sigmas,
                               print_covariance}
    {}

    auto operator()(std::unique_ptr<SolverFactoryInterface<TModel>> solver_factory)
    {
      // Prepare
      this->pose3_sampler_ = gtsam::Sampler{this->pose3_sampler_sigmas_, 42u};
      this->point2_sampler_ = gtsam::Sampler{this->point2_sampler_sigmas_, 42u};
      this->frames_processed_ = 0;

      gttic(solver);

      // Instantiate the solver
      auto solver{solver_factory->new_solver(*this)};

      // Collect all perturbed values at this level. The solver can use them or not
      // but the random number generator generates the same series for each solver.

      // Loop over all the cameras
      for (auto &camera : this->model_.cameras_.cameras_) {
        std::vector<BoardData<TModel>> board_datas{};

        // And loop over boards
        for (auto &board : this->model_.boards_.boards_) {
          BoardData<TModel>::load_board_datas(*this, camera, board, board_datas);
        }

        // Let the solver work on these measurements.
        FrameData<TModel> frame_data{camera,
                                     this->get_perturbed_camera_f_world(camera),
                                     board_datas};
        solver->add_frame(frame_data);
        this->frames_processed_ += 1;
      }

      // Call the solver and get the result.
      auto result = solver->solve();

      gttoc(solver);
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();

      return result;
    }
  };

  using CheckerboardSolverRunner = SolverRunner<CheckerboardCalibrationModel>; //
  using CharucoboardSolverRunner = SolverRunner<CharucoboardCalibrationModel>; //
}

#endif //_CAL_SOLVER_RUNNER_HPP
