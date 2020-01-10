

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
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace camsim
{
  struct MarkerModelRef
  {
    const MarkerModel &marker_;

    MarkerModelRef(const MarkerModel &marker) :
      marker_{marker}
    {};
  };

  struct SolverRunner
  {
    typedef std::function<void(const CameraModel &,
                               const std::vector<MarkerModelRef> &,
                               gtsam::Sampler &)> AddFrameMeasurementsFunc;

    typedef std::function<AddFrameMeasurementsFunc(SolverRunner &)> SolverFactoryFunc;

    const Model &model_;
    const gtsam::Vector6 sampler_sigmas_;
    const bool print_covariance_;

    const gtsam::SharedNoiseModel noise_;

    int frames_processed_{0};

    SolverRunner(const Model &model,
                 const gtsam::Vector6 &sampler_sigmas,
                 const gtsam::Vector6 &noise_sigmas,
                 bool print_covariance) :
      model_{model},
      sampler_sigmas_{std::move(sampler_sigmas)},
      print_covariance_{print_covariance},
      noise_{gtsam::noiseModel::Diagonal::Sigmas(noise_sigmas)}
    {}

    void operator()(const SolverFactoryFunc &solver_factory)
    {
      frames_processed_ = 0;
      auto solver{solver_factory(*this)};
      gtsam::Sampler sampler{sampler_sigmas_, 42u};

      for (auto &camera : model_.cameras_.cameras_) {
        std::vector<MarkerModelRef> marker_refs{};

        for (auto &marker : model_.markers_.markers_) {
          if (!model_.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_.empty()) {
            auto camera_f_marker = marker.marker_f_world_.inverse() * camera.camera_f_world_;
            marker_refs.emplace_back(MarkerModelRef{marker});
          }
        }

        if (marker_refs.size() > 1) {
          solver(camera, marker_refs, sampler);
          frames_processed_ += 1;
        }
      }
    }

    void display_results(const gtsam::Pose3 &pose, const gtsam::Matrix &cov) const
    {
      std::cout << PoseWithCovariance::to_str(pose) << std::endl;
      if (print_covariance_) {
        std::cout << PoseWithCovariance::to_str(gtsam::Matrix6{cov}) << std::endl;
      }
    }

    gtsam::Pose3 get_camera_f_marker(const CameraModel &camera, const MarkerModel &marker) const
    {
      return marker.marker_f_world_.inverse() * camera.camera_f_world_;
    }

    std::vector<gtsam::Point2> get_corners_f_images(const CameraModel &camera, const MarkerModel &marker) const
    {
      return model_.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;
    }
  };

  class SolverGlobal
  {
    const SolverRunner &sr_;
    const bool auto_initial_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". I can't figure out how to avoid the copies when they are passed
      // around as functors.
      if (graph_.empty()) {
        return;
      }

      if (auto_initial_) {
        initial_ = gtsam::InitializePose3::initialize(graph_);
      }

      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      gtsam::Marginals marginals{graph_, result};
      for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(m.value.cast<gtsam::Pose3>(),
                            marginals.marginalCovariance(m.key));
      }
    }

    void add_priors()
    {
      // Add the prior for marker 0
      gtsam::Symbol marker_key{'m', 0};
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
      graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                               sr_.model_.markers_.markers_[0].marker_f_world_,
                                                               priorModel);
    }

  public:
    explicit SolverGlobal(SolverRunner &sr, bool auto_initial) :
      sr_{sr}, auto_initial_{auto_initial}
    {}

    ~SolverGlobal()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs,
                    gtsam::Sampler &sampler)
    {
      if (sr_.frames_processed_ == 0) {
        add_priors();
      }

      gtsam::Symbol camera_key{'c', static_cast<std::uint64_t>(camera.camera_idx_)};

      // Add the initial camera pose estimate
      if (!auto_initial_) {
        initial_.insert(camera_key, camera.camera_f_world_.retract(sampler.sample()));
      }

      for (auto marker_ref : marker_refs) {
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(marker_ref.marker_.marker_idx_)};

        // Add the measurement factor.
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          sr_.get_camera_f_marker(camera, marker_ref.marker_).retract(sampler.sample()),
          sr_.noise_);

        // Add the initial marker value estimate
        if (!auto_initial_ && !initial_.exists(marker_key)) {
          initial_.insert(marker_key, marker_ref.marker_.marker_f_world_.retract(sampler.sample()));
        }
      }
    }
  };

  class SolverGlobalSFM
  {
    const SolverRunner &sr_;

    gtsam::NonlinearFactorGraph graph_slam_{};
    gtsam::NonlinearFactorGraph graph_sfm_{};

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". I can't figure out how to avoid the copies when they are passed
      // around as functors.
      if (graph_slam_.empty()) {
        return;
      }

      auto initial = gtsam::InitializePose3::initialize(graph_slam_);

      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_slam_, initial, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_slam_.error(initial) << std::endl;
      std::cout << "final error = " << graph_slam_.error(result) << std::endl;

      gtsam::Marginals marginals{graph_slam_, result};
      for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(m.value.cast<gtsam::Pose3>(),
                            marginals.marginalCovariance(m.key));
      }
    }

    void add_priors()
    {
      // Add the prior for marker 0
      gtsam::Symbol marker_key{'m', 0};
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
      graph_slam_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                                    sr_.model_.markers_.markers_[0].marker_f_world_,
                                                                    priorModel);
    }

    PoseWithCovariance calc_camera_f_marker(std::vector<gtsam::Point2> &corners_f_image)
    {
    }

    PoseWithCovariance calc_marker_f_world(std::vector<PointWithCovariance> &corners_f_world)
    {

    }


  public:
    explicit SolverGlobalSFM(SolverRunner &sr) :
      sr_{sr}
    {}

    ~SolverGlobalSFM()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs,
                    gtsam::Sampler &sampler)
    {
      if (sr_.frames_processed_ == 0) {
        add_priors();
      }

      gtsam::Symbol camera_key{'c', static_cast<std::uint64_t>(camera.camera_idx_)};

      for (auto &marker_ref : marker_refs) {
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(marker_ref.marker_.marker_idx_)};

        // Add the measurement factor.
        graph_slam_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          sr_.get_camera_f_marker(camera, marker_ref.marker_).retract(sampler.sample()), sr_.noise_);
      }
    }
  };

  class SolverISAM
  {
    const SolverRunner &sr_;

    gtsam::ISAM2 isam2_{get_isam2_params()};
    std::map<int, int> marker_seen_counts_{};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeThreshold = 0.01;
      isam2_params.relinearizeSkip = 1;
      return isam2_params;
    }

    void display_results()
    {
      if (isam2_.empty()) {
        return;
      }

      gtsam::Values currentEstimate = isam2_.calculateEstimate();
      std::cout << "****************************************************" << std::endl;
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      for (auto m : currentEstimate.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(m.value.cast<gtsam::Pose3>(),
                            isam2_.marginalCovariance(m.key));
      }
    }

    void add_priors(gtsam::NonlinearFactorGraph &graph)
    {
      // Add the prior for marker 0
      gtsam::Symbol marker_key{'m', 0};
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(marker_key,
                                                              sr_.model_.markers_.markers_[0].marker_f_world_,
                                                              priorModel);
    }

  public:
    explicit SolverISAM(SolverRunner &sr) :
      sr_{sr}
    {}

    ~SolverISAM()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs,
                    gtsam::Sampler &sampler)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      if (sr_.frames_processed_ == 0) {
        add_priors(graph);
      }

      gtsam::Symbol camera_key{'c', static_cast<std::uint64_t>(camera.camera_idx_)};

      // Add the initial camera pose estimate
      initial.insert(camera_key, camera.camera_f_world_.retract(sampler.sample()));

      for (auto &marker_ref : marker_refs) {
        gtsam::Symbol marker_key{'m', static_cast<std::uint64_t>(marker_ref.marker_.marker_idx_)};

        // Add the measurement factor.
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          sr_.get_camera_f_marker(camera, marker_ref.marker_).retract(sampler.sample()), sr_.noise_);

        // update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_ref.marker_.marker_idx_);
        if (pair == marker_seen_counts_.end()) {
          marker_seen_counts_.insert(std::pair<int, int>{marker_ref.marker_.marker_idx_, 1});
        } else {
          pair->second += 1;
        }

        // Add the initial marker value estimate only if this marker has not been seen.
        if (pair == marker_seen_counts_.end()) {
          std::cout << "Adding marker " << marker_ref.marker_.marker_idx_
                    << " at frame " << sr_.frames_processed_ + 1 << std::endl;
          initial.insert(marker_key, marker_ref.marker_.marker_f_world_.retract(sampler.sample()));
        }
      }

      // Update iSAM with the new factors
      isam2_.update(graph, initial);
      isam2_.update();
    }
  };


  void map_global()
  {
    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingOrigin{8, 2.},
                            PoseGens::SpinAboutZAtOriginFacingOut{256},
                            camsim::CameraTypes::simulation,
                            0.1775}};

    double r_sigma = 0.1;
    double t_sigma = 0.3;

    SolverRunner solver_runner{model,
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                 gtsam::Vector3::Constant(t_sigma)).finished(),
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                 gtsam::Vector3::Constant(t_sigma)).finished(),
                               false};

    solver_runner([](SolverRunner &solver_runner)
                  { return SolverGlobal{solver_runner, false}; });

    solver_runner([](SolverRunner &solver_runner)
                  { return SolverGlobal{solver_runner, true}; });

    solver_runner([](SolverRunner &solver_runner)
                  { return SolverISAM{solver_runner}; });

    solver_runner([](SolverRunner &solver_runner)
                  { return SolverGlobalSFM{solver_runner}; });
  }
}