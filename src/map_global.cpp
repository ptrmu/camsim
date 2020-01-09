

#include "map_run.hpp"
#include "model.hpp"
#include "pose_with_covariance.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include "gtsam/linear/Sampler.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace camsim
{
  struct MeasurementFactor
  {
    const MarkerModel &marker_;
    const gtsam::Pose3 camera_f_marker_;

    MeasurementFactor(const MarkerModel &marker, gtsam::Pose3 camera_f_marker) :
      marker_{marker}, camera_f_marker_{std::move(camera_f_marker)}
    {}
  };

  class SolveGlobal
  {
    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};
    int frames_processed_{0};
    bool loop_closed{false};

    void display_results()
    {
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_).optimize();
      std::cout << "Frame " << frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      gtsam::Marginals marginals{graph_, result};
      for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
        std::cout << PoseWithCovariance::to_str(m.value.cast<gtsam::Pose3>()) << std::endl;
        std::cout << PoseWithCovariance::to_str(gtsam::Matrix6{marginals.marginalCovariance(m.key)}) << std::endl;
      }
    }

  public:
    void add_frame_measurements(const CameraModel &camera,
                                const std::vector<MeasurementFactor> &measurement_factors,
                                gtsam::SharedNoiseModel &noise,
                                gtsam::Sampler &sampler)
    {
      if (!loop_closed &&
          measurement_factors.size() == 2 &&
          measurement_factors[0].marker_.marker_idx_ == 0 &&
          measurement_factors[1].marker_.marker_idx_ == 7) {
        loop_closed = true;
        display_results();
      }

      gtsam::Symbol camera_key{'c', static_cast<std::uint64_t>(camera.camera_idx_)};

      // Add the initial camera pose estimate
      initial_.insert(camera_key, camera.camera_f_world_.retract(sampler.sample()));

      for (auto &mf : measurement_factors) {
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(mf.marker_.marker_idx_)};

        // Add the measurement factor.
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          mf.camera_f_marker_.retract(sampler.sample()), noise);

        // Add the initial marker value estimate
        if (!initial_.exists(marker_key)) {
          initial_.insert(marker_key, mf.marker_.marker_f_world_.retract(sampler.sample()));
        }

        // Add the prior if this is marker 0
        if (mf.marker_.marker_idx_ == 0) {
          static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
          graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                                   mf.marker_.marker_f_world_,
                                                                   priorModel);
        }
      }

      frames_processed_ += 1;
    }

    void finish()
    {
      display_results();
    }
  };


  class SolveISAM
  {
    gtsam::ISAM2 isam2_{get_isam2_params()};
    std::map<int, int> marker_seen_counts_{};
    int frames_processed_{0};
    bool loop_closed{false};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeThreshold = 0.01;
      isam2_params.relinearizeSkip = 1;
      return isam2_params;
    }

    void display_results()
    {
      gtsam::Values currentEstimate = isam2_.calculateEstimate();
      std::cout << "****************************************************" << std::endl;
      std::cout << "Frame " << frames_processed_ << ": " << std::endl;
      for (auto m : currentEstimate.filter(gtsam::Symbol::ChrTest('m'))) {
        std::cout << PoseWithCovariance::to_str(m.value.cast<gtsam::Pose3>()) << std::endl;
        std::cout << PoseWithCovariance::to_str(gtsam::Matrix6{isam2_.marginalCovariance(m.key)}) << std::endl;
      }
    }

  public:
    void add_frame_measurements(const CameraModel &camera,
                                const std::vector<MeasurementFactor> &measurement_factors,
                                gtsam::SharedNoiseModel &noise,
                                gtsam::Sampler &sampler)
    {
      if (!loop_closed &&
          measurement_factors.size() == 2 &&
          measurement_factors[0].marker_.marker_idx_ == 0 &&
          measurement_factors[1].marker_.marker_idx_ == 7) {
        loop_closed = true;
        display_results();
      }

      gtsam::Symbol camera_key{'c', static_cast<std::uint64_t>(camera.camera_idx_)};

      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Add the initial camera pose estimate
      initial.insert(camera_key, camera.camera_f_world_.retract(sampler.sample()));

      for (auto &mf : measurement_factors) {
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(mf.marker_.marker_idx_)};

        // Add the measurement factor.
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          mf.camera_f_marker_.retract(sampler.sample()), noise);

        // update the marker seen counts
        auto pair = marker_seen_counts_.find(mf.marker_.marker_idx_);
        if (pair == marker_seen_counts_.end()) {
          marker_seen_counts_.insert(std::pair<int, int>{mf.marker_.marker_idx_, 1});
        } else {
          pair->second += 1;
        }

        // Add the initial marker value estimate only if this marker has not been seen.
        if (pair == marker_seen_counts_.end()) {
          std::cout << "Adding marker " << mf.marker_.marker_idx_ << " at frame " << frames_processed_ + 1 << std::endl;
          initial.insert(marker_key, mf.marker_.marker_f_world_.retract(sampler.sample()));
        }

        // Add the prior if this is marker 0
        if (mf.marker_.marker_idx_ == 0) {
          static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
          graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                                  mf.marker_.marker_f_world_,
                                                                  priorModel);
        }
      }

      // Update iSAM with the new factors
      isam2_.update(graph, initial);
      isam2_.update();

      frames_processed_ += 1;
    }

    void finish()
    {
      display_results();
    }
  };

  template<class TSolve>
  void run(Model &model, TSolve &solver, gtsam::SharedNoiseModel &noise, gtsam::Sampler &sampler)
  {
    for (auto &camera : model.cameras_.cameras_) {
      std::vector<MeasurementFactor> measurement_factors{};

      for (auto &marker : model.markers_.markers_) {
        if (!model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_.empty()) {
          auto camera_f_marker = marker.marker_f_world_.inverse() * camera.camera_f_world_;
          measurement_factors.emplace_back(MeasurementFactor{marker, camera_f_marker});
        }
      }

      if (measurement_factors.size() > 1) {
        solver.add_frame_measurements(camera, measurement_factors, noise, sampler);
      }
    }

    solver.finish();
  }

  void map_global()
  {
    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingOrigin{8, 2.},
                            PoseGens::SpinAboutZAtOriginFacingOut{4096},
                            camsim::CameraTypes::simulation,
                            0.1775}};

    double r_sigma = 0.1;
    double t_sigma = 0.3;

    auto sampler_dist = (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
      gtsam::Vector3::Constant(t_sigma)).finished();;

    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << gtsam::Vector3::Constant(r_sigma),
        gtsam::Vector3::Constant(t_sigma)).finished());

    SolveGlobal solve_global{};

    SolveISAM solve_isam{};

    gtsam::Sampler sampler_global{sampler_dist, 42u};
    run(model, solve_global, noise, sampler_global);

    gtsam::Sampler sampler_isam{sampler_dist, 42u};
    run(model, solve_isam, noise, sampler_isam);
  }
}