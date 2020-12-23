
#ifndef _MAP_SOLVER_RUNNER_HPP
#define _MAP_SOLVER_RUNNER_HPP

#include "../model.hpp"
#include "../pose_with_covariance.hpp"

#define ENABLE_TIMING

#include <gtsam/base/timing.h>
#include <gtsam/linear/NoiseModel.h>
#include "gtsam/linear/Sampler.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>

namespace camsim
{
  typedef std::reference_wrapper<const MarkerModel> MarkerModelRef;

  struct MarkerData
  {
    const MarkerModel &marker_;
    const gtsam::Pose3 marker_f_world_perturbed_;
    const gtsam::Pose3 camera_f_marker_perturbed_;
    const std::vector<gtsam::Point2> corners_f_image_perturbed_;
    const gtsam::Pose3 camera_f_marker_;
    const std::vector<gtsam::Point2> corners_f_image_;

    MarkerData(const MarkerModel &marker,
               gtsam::Pose3 marker_f_world_perturbed,
               gtsam::Pose3 camera_f_marker_perturbed,
               std::vector<gtsam::Point2> corners_f_image_perturbed,
               gtsam::Pose3 camera_f_marker,
               std::vector<gtsam::Point2> corners_f_image) :
      marker_{marker},
      marker_f_world_perturbed_{marker_f_world_perturbed},
      camera_f_marker_perturbed_{camera_f_marker_perturbed},
      corners_f_image_perturbed_{corners_f_image_perturbed},
      camera_f_marker_{camera_f_marker},
      corners_f_image_{corners_f_image}
    {}
  };

  struct FrameData
  {
    const CameraModel &camera_;
    const gtsam::Pose3 camera_f_world_perturbed_;
    const std::vector<MarkerData> marker_datas_;

    FrameData(const CameraModel &camera,
              const gtsam::Pose3 camera_f_world_perturbed,
              const std::vector<MarkerData> marker_datas) :
      camera_{camera},
      camera_f_world_perturbed_{camera_f_world_perturbed},
      marker_datas_{marker_datas}
    {}
  };

  struct SolverRunner
  {
    typedef std::function<void(const FrameData &)> AddFrameMeasurementsFunc;

    typedef std::function<AddFrameMeasurementsFunc(SolverRunner &)> SolverFactoryFunc;

    const Model &model_;
    const gtsam::Vector6 pose3_sampler_sigmas_;
    const gtsam::Vector2 point2_sampler_sigmas_;
    const bool print_covariance_;

    const gtsam::SharedNoiseModel pose3_noise_;
    const gtsam::SharedNoiseModel point2_noise_;
    const gtsam::SharedNoiseModel point2x4_noise_;

    gtsam::Sampler pose3_sampler_;
    gtsam::Sampler point2_sampler_;
    int frames_processed_{0};

    SolverRunner(const Model &model,
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
      point2x4_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas.replicate<4, 1>())},
      pose3_sampler_(pose3_sampler_sigmas_),
      point2_sampler_(point2_sampler_sigmas_)
    {}

    void operator()(const SolverFactoryFunc &solver_factory)
    {
      // Prepare
      pose3_sampler_ = gtsam::Sampler{pose3_sampler_sigmas_, 42u};
      point2_sampler_ = gtsam::Sampler{point2_sampler_sigmas_, 42u};
      frames_processed_ = 0;

      gttic(solver);

      {
        // Instantiate the solver
        auto solver{solver_factory(*this)};

        // Collect all perturbed values at this level. The solver can use them or not
        // but the random number generator generates the same series for each solver.

        // Loop over all the cameras
        for (auto &camera : model_.cameras_.cameras_) {
          std::vector<MarkerData> marker_datas{};

          // Figure out which markers are visible from this camera
          for (auto &marker : model_.markers_.markers_) {
            if (!model_.corners_f_images_[camera.index()][marker.index()].corners_f_image_.empty()) {
              marker_datas.emplace_back(MarkerData{marker,
                                                   get_perturbed_marker_f_world(marker),
                                                   get_perturbed_camera_f_marker(camera, marker),
                                                   get_perturbed_corners_f_image(camera, marker),
                                                   get_camera_f_marker(camera, marker),
                                                   get_corners_f_image(camera, marker)});
            }
          }

          // Let the solver work on these measurements.
          if (marker_datas.size() > 1) {
            FrameData frame_data{camera,
                                 get_perturbed_camera_f_world(camera),
                                 marker_datas};
            solver(frame_data);
            frames_processed_ += 1;
          }
        }
      }

      gttoc(solver);
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();
    }

    void display_results(const PoseWithCovariance &pose_f_world) const
    {
      gtsam::Symbol sym{pose_f_world.key_};
      std::cout << sym.chr() << sym.index() << PoseWithCovariance::to_str(pose_f_world.pose_)
                << " std: " << PoseWithCovariance::to_stddev_str(pose_f_world.cov_)
                << std::endl;
      if (print_covariance_) {
        std::cout << PoseWithCovariance::to_matrix_str(pose_f_world.cov_, true) << std::endl;
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

  private:
    gtsam::Pose3 get_perturbed_camera_f_world(const CameraModel &camera)
    {
      return camera.camera_f_world_.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_perturbed_marker_f_world(const MarkerModel &marker)
    {
      return marker.marker_f_world_.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_camera_f_marker(const CameraModel &camera,
                                     const MarkerModel &marker)
    {
      return (marker.marker_f_world_.inverse() * camera.camera_f_world_);
    }

    gtsam::Pose3 get_perturbed_camera_f_marker(const CameraModel &camera,
                                               const MarkerModel &marker)
    {
      return get_camera_f_marker(camera, marker).retract(pose3_sampler_.sample());
    }

    std::vector<gtsam::Point2> get_corners_f_image(const CameraModel &camera,
                                                   const MarkerModel &marker)
    {
      return model_.corners_f_images_[camera.index()][marker.index()].corners_f_image_;
    }

    std::vector<gtsam::Point2> get_perturbed_corners_f_image(const CameraModel &camera,
                                                             const MarkerModel &marker)
    {
      auto corners_f_image{get_corners_f_image(camera, marker)};
      for (auto &corner_f_image : corners_f_image) {
        corner_f_image += point2_sampler_.sample();
      }
      return corners_f_image;
    }

  public:
    void add_marker_0_prior(gtsam::NonlinearFactorGraph &graph, gtsam::Values &initial)
    {
      add_marker_0_prior(graph);
      initial.insert(model_.markers_.markers_[0].key_,
                     model_.markers_.markers_[0].marker_f_world_);
    }

    void add_marker_0_prior(gtsam::NonlinearFactorGraph &graph)
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
}

#endif //_MAP_SOLVER_RUNNER_HPP
