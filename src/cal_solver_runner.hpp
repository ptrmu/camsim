
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

  template<typename TCalibrationModel>
  struct SolverRunnerBase
  {
    const TCalibrationModel &model_;
    const gtsam::Vector6 pose3_sampler_sigmas_;
    const gtsam::Vector2 point2_sampler_sigmas_;
    const bool print_covariance_;

    const gtsam::SharedNoiseModel pose3_noise_;
    const gtsam::SharedNoiseModel point2_noise_;
    const gtsam::SharedNoiseModel point2x4_noise_;

    gtsam::Sampler pose3_sampler_{};
    gtsam::Sampler point2_sampler_{};
    int frames_processed_{0};

    SolverRunnerBase(const TCalibrationModel &model,
                     const gtsam::Vector6 &pose3_sampler_sigmas,
                     const gtsam::Vector6 &pose3_noise_sigmas,
                     const gtsam::Vector2 &point2_sampler_sigmas,
                     const gtsam::Vector2 &point2_noise_sigmas,
                     bool print_covariance) :
      model_{model},
      pose3_sampler_sigmas_{std::move(pose3_sampler_sigmas)},
      point2_sampler_sigmas_{std::move(point2_sampler_sigmas)},
      print_covariance_{print_covariance},
      pose3_noise_{gtsam::noiseModel::Diagonal::Sigmas(pose3_noise_sigmas)},
      point2_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas)},
      point2x4_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas.replicate<4, 1>())}
    {}

    void display_results(const PoseWithCovariance &pose_f_world) const
    {
      gtsam::Symbol sym{pose_f_world.key_};
      std::cout << sym.chr() << sym.index() << PoseWithCovariance::to_str(pose_f_world.pose_)
                << " std: " << PoseWithCovariance::to_stddev_str(pose_f_world.cov_)
                << std::endl;
      if (print_covariance_) {
        std::cout << PoseWithCovariance::to_str(pose_f_world.cov_) << std::endl;
      }
    }

    void display_results(const std::array<camsim::PointWithCovariance, 4> &corners_f_world) const
    {
      std::cout << "m" << gtsam::Symbol(corners_f_world[0].key_).index() << std::endl;
      for (auto const &corner_f_world : corners_f_world) {
        std::cout << PointWithCovariance::to_str(corner_f_world.point_)
                  << " std: " << PointWithCovariance::to_stddev_str(corner_f_world.cov_)
                  << std::endl;
        if (print_covariance_) {
          std::cout << PointWithCovariance::to_str(corner_f_world.cov_) << std::endl;
        }
      }
    }

    gtsam::Pose3 get_perturbed_camera_f_world(const CameraModel &camera)
    {
      return camera.camera_f_world_.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_perturbed_board_f_world(const typename TCalibrationModel::BoardModel &board)
    {
      return board.board.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_camera_f_board(const CameraModel &camera,
                                    const MarkerModel &board)
    {
      return (board.marker_f_world_.inverse() * camera.camera_f_world_);
    }

    gtsam::Pose3 get_perturbed_camera_f_board(const CameraModel &camera,
                                              const MarkerModel &board)
    {
      return get_camera_f_board(camera, board).retract(pose3_sampler_.sample());
    }

    std::vector<gtsam::Point2> get_corners_f_image(const CameraModel &camera,
                                                   const MarkerModel &board)
    {
      return model_.corners_f_images_[camera.index()][board.index()].corners_f_image_;
    }

    std::vector<gtsam::Point2> get_perturbed_corners_f_image(const CameraModel &camera,
                                                             const MarkerModel &board)
    {
      auto corners_f_image{get_corners_f_image(camera, board)};
      for (auto &corner_f_image : corners_f_image) {
        corner_f_image = corner_f_image.retract(point2_sampler_.sample());
      }
      return corners_f_image;
    }

    void add_board_0_prior(gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
    {
      add_board_0_prior(graph);
      initial.insert(model_.markers_.markers_[0].key_,
                     model_.markers_.markers_[0].marker_f_world_);
    }

    void add_board_0_prior(gtsam::NonlinearFactorGraph &graph)
    {
      // Add the prior for marker 0
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(model_.markers_.markers_[0].key_,
                                                              model_.markers_.markers_[0].marker_f_world_,
                                                              priorModel);
    }

    std::unique_ptr<gtsam::Marginals> get_marginals(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values)
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

  template<typename TCalibrationModel>
  struct BoardData
  {
    const typename TCalibrationModel::BoardModel &board_;
    const gtsam::Pose3 camera_f_board_;
    const std::vector<JunctionFImage> junctions_f_image_;
    const std::vector<JunctionFImage> junctions_f_image_perturbed_;

    BoardData(const typename TCalibrationModel::BoardModel &board,
              const SolverRunnerBase<TCalibrationModel> &solver_runner_calculator);
  };

  struct CheckerboardData : public BoardData<CheckerboardCalibrationModel>
  {
    CheckerboardData(const CheckerboardModel &board,
                     const SolverRunnerBase<CheckerboardCalibrationModel> &solver_runner_calculator);
  };

  struct CharucoboardData : public BoardData<CharucoboardCalibrationModel>
  {
    std::vector<ArucoCornersFImage> arucos_corners_f_image_;
    std::vector<ArucoCornersFImage> arucos_corners_f_image_perturbed_;

    CharucoboardData(const CharucoboardModel &board,
                     const SolverRunnerBase<CharucoboardCalibrationModel> &solver_runner_calculator);
  };

//  static void load_board_datas(SolverRunnerBase<TCalibrationModel> sr,
//                               const TCalibrationModel &model,
//                               const CameraModel &camera,
//                               const CheckerboardModel &board,
//                               std::vector<BoardData<TCalibrationModel>> board_datas);

// ==============================================================================
// FrameData
// ==============================================================================

  template<typename TCalibrationModel>
  struct FrameData
  {
    const CameraModel &camera_;
    const gtsam::Pose3 camera_f_world_perturbed_;
    const std::vector<BoardData<TCalibrationModel>> board_datas_;

    FrameData(const CameraModel &camera,
              const gtsam::Pose3 camera_f_world_perturbed,
              const std::vector<BoardData<TCalibrationModel>> board_datas) :
      camera_{camera},
      camera_f_world_perturbed_{camera_f_world_perturbed},
      board_datas_{board_datas}
    {}
  };

// ==============================================================================
// SolverInterface
// ==============================================================================

  template<typename TCalibrationModel>
  struct SolverInterface
  {
    virtual ~SolverInterface() = default; //
    virtual void add_frame(const FrameData<TCalibrationModel> &frame_data) = 0; //
  };

  template<typename TCalibrationModel>
  struct SolverFactoryInterface
  {
    virtual ~SolverFactoryInterface() = default; //
    virtual std::unique_ptr<SolverInterface<TCalibrationModel>> new_solver(
      const SolverRunnerBase<TCalibrationModel> &solver_runner) = 0; //
  };

// ==============================================================================
// SolverRunner
// ==============================================================================

//  template<typename TCalibrationModel>
//  struct SolverRunnerBase
//  {
//    const Model &model_;
//    const gtsam::Vector6 pose3_sampler_sigmas_;
//    const gtsam::Vector2 point2_sampler_sigmas_;
//    const bool print_covariance_;
//
//    const gtsam::SharedNoiseModel pose3_noise_;
//    const gtsam::SharedNoiseModel point2_noise_;
//    const gtsam::SharedNoiseModel point2x4_noise_;
//
//    gtsam::Sampler pose3_sampler_{};
//    gtsam::Sampler point2_sampler_{};
//    int frames_processed_{0};
//
//    SolverRunnerBase(const Model &model,
//                     const gtsam::Vector6 &pose3_sampler_sigmas,
//                     const gtsam::Vector6 &pose3_noise_sigmas,
//                     const gtsam::Vector2 &point2_sampler_sigmas,
//                     const gtsam::Vector2 &point2_noise_sigmas,
//                     bool print_covariance) :
//      model_{model},
//      pose3_sampler_sigmas_{std::move(pose3_sampler_sigmas)},
//      point2_sampler_sigmas_{std::move(point2_sampler_sigmas)},
//      print_covariance_{print_covariance},
//      pose3_noise_{gtsam::noiseModel::Diagonal::Sigmas(pose3_noise_sigmas)},
//      point2_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas)},
//      point2x4_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas.replicate<4, 1>())}
//    {}
//
//    void display_results(const PoseWithCovariance &pose_f_world) const
//    {
//      gtsam::Symbol sym{pose_f_world.key_};
//      std::cout << sym.chr() << sym.index() << PoseWithCovariance::to_str(pose_f_world.pose_)
//                << " std: " << PoseWithCovariance::to_stddev_str(pose_f_world.cov_)
//                << std::endl;
//      if (print_covariance_) {
//        std::cout << PoseWithCovariance::to_str(pose_f_world.cov_) << std::endl;
//      }
//    }
//
//    void display_results(const std::array<camsim::PointWithCovariance, 4> &corners_f_world) const
//    {
//      std::cout << "m" << gtsam::Symbol(corners_f_world[0].key_).index() << std::endl;
//      for (auto const &corner_f_world : corners_f_world) {
//        std::cout << PointWithCovariance::to_str(corner_f_world.point_)
//                  << " std: " << PointWithCovariance::to_stddev_str(corner_f_world.cov_)
//                  << std::endl;
//        if (print_covariance_) {
//          std::cout << PointWithCovariance::to_str(corner_f_world.cov_) << std::endl;
//        }
//      }
//    }
//
//    gtsam::Pose3 get_perturbed_camera_f_world(const CameraModel &camera)
//    {
//      return camera.camera_f_world_.retract(pose3_sampler_.sample());
//    }
//
//    gtsam::Pose3 get_perturbed_board_f_world(const MarkerModel &board)
//    {
//      return board.marker_f_world_.retract(pose3_sampler_.sample());
//    }
//
//    gtsam::Pose3 get_camera_f_board(const CameraModel &camera,
//                                    const MarkerModel &board)
//    {
//      return (board.marker_f_world_.inverse() * camera.camera_f_world_);
//    }
//
//    gtsam::Pose3 get_perturbed_camera_f_board(const CameraModel &camera,
//                                              const MarkerModel &board)
//    {
//      return get_camera_f_board(camera, board).retract(pose3_sampler_.sample());
//    }
//
//    std::vector<gtsam::Point2> get_corners_f_image(const CameraModel &camera,
//                                                   const MarkerModel &board)
//    {
//      return model_.corners_f_images_[camera.index()][board.index()].corners_f_image_;
//    }
//
//    std::vector<gtsam::Point2> get_perturbed_corners_f_image(const CameraModel &camera,
//                                                             const MarkerModel &board)
//    {
//      auto corners_f_image{get_corners_f_image(camera, board)};
//      for (auto &corner_f_image : corners_f_image) {
//        corner_f_image = corner_f_image.retract(point2_sampler_.sample());
//      }
//      return corners_f_image;
//    }
//
//    void add_board_0_prior(gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
//    {
//      add_board_0_prior(graph);
//      initial.insert(model_.markers_.markers_[0].key_,
//                     model_.markers_.markers_[0].marker_f_world_);
//    }
//
//    void add_board_0_prior(gtsam::NonlinearFactorGraph &graph)
//    {
//      // Add the prior for marker 0
//      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
//      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(model_.markers_.markers_[0].key_,
//                                                              model_.markers_.markers_[0].marker_f_world_,
//                                                              priorModel);
//    }
//
//    std::unique_ptr<gtsam::Marginals> get_marginals(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values)
//    {
//      try {
//        return std::make_unique<gtsam::Marginals>(graph, values, gtsam::Marginals::QR);
//      }
//      catch (gtsam::IndeterminantLinearSystemException &ex) {
//      }
//      std::cout << "Could not create marginals, trying QR" << std::endl;
//
//      try {
//        return std::make_unique<gtsam::Marginals>(graph, values, gtsam::Marginals::QR);
//      }
//      catch (gtsam::IndeterminantLinearSystemException &ex) {
//      }
//      std::cout << "Could not create marginals, returning null" << std::endl;
//
//      return std::unique_ptr<gtsam::Marginals>{};
//    }
//  };

  template<typename TCalibrationModel>
  struct SolverRunner : public SolverRunnerBase<TCalibrationModel>
  {
    SolverRunner(const TCalibrationModel &model,
                 const gtsam::Vector6 &pose3_sampler_sigmas,
                 const gtsam::Vector6 &pose3_noise_sigmas,
                 const gtsam::Vector2 &point2_sampler_sigmas,
                 const gtsam::Vector2 &point2_noise_sigmas,
                 bool print_covariance) :
      SolverRunnerBase<TCalibrationModel>{model,
                                          pose3_sampler_sigmas, pose3_noise_sigmas,
                                          point2_sampler_sigmas, point2_noise_sigmas,
                                          print_covariance}
    {}

    void operator()(std::unique_ptr<SolverFactoryInterface<TCalibrationModel>> solver_factory)
    {
      // Prepare
      this->pose3_sampler_ = gtsam::Sampler{this->pose3_sampler_sigmas_, 42u};
      this->point2_sampler_ = gtsam::Sampler{this->point2_sampler_sigmas_, 42u};
      this->frames_processed_ = 0;

      gttic(solver);

      {
        // Instantiate the solver
        auto solver{solver_factory->new_solver(*this)};

        // Collect all perturbed values at this level. The solver can use them or not
        // but the random number generator generates the same series for each solver.

        // Loop over all the cameras
        for (auto &camera : this->model_.cameras_.cameras_) {
          std::vector<BoardData<TCalibrationModel>> board_datas{};

          // Figure out which boards have junctions visible from this camera
          for (auto &board : this->model_.boards_.boards_) {
//            load_board_datas(*this, this->model_, camera, board, board_datas);
//            auto &cb_junctions_f_image = this->model_.junctions_f_images_[camera.index()][board.index()];
//
//            if (!this->model_.junctions_f_images_[camera.index()][board.index()].corners_f_image_.empty()) {
//              board_datas.emplace_back(BoardData<TCalibrationModel>{
//                board,
//                get_perturbed_board_f_world(board),
//                get_perturbed_camera_f_board(camera, board),
//                get_perturbed_corners_f_image(camera, board),
//                get_camera_f_board(camera, board),
//                get_corners_f_image(camera, board)});
//            }
          }

          // Let the solver work on these measurements.
          if (board_datas.size() > 1) {
            FrameData<TCalibrationModel> frame_data{camera,
                                                    this->get_perturbed_camera_f_world(camera),
                                                    board_datas};
            solver->add_frame(frame_data);
            this->frames_processed_ += 1;
          }
        }
      }

      gttoc(solver);
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();
    }
  };

  using CheckerboardSolverRunner = SolverRunner<CheckerboardCalibrationModel>; //
  using CharucoboardSolverRunner = SolverRunner<CharucoboardCalibrationModel>; //
}

#endif //_CAL_SOLVER_RUNNER_HPP
