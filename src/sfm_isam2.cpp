
#include "sfm_isam2.hpp"

#include <set>
#include <map>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace camsim
{
  static gtsam::ISAM2Params get_isam2_parameters()
  {
    gtsam::ISAM2Params parameters;

    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    return parameters;
  }

  class SfmIsam2Impl
  {
    const int key_marker_id_;
    const gtsam::Pose3 key_marker_f_world_;

    std::set<int> markers_seen_{};
    gtsam::ISAM2 isam_{get_isam2_parameters()};

    std::map<int, PoseWithCovariance> markers_known_{};
    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initialEstimate_{};

  public:
    SfmIsam2Impl(int key_marker_id, const gtsam::Pose3 &key_marker_f_world) :
      key_marker_id_{key_marker_id}, key_marker_f_world_{key_marker_f_world}
    {
    }

    void add_measurements(int camera_id, const std::vector<PoseWithCovariance> &camera_f_markers)
    {
//      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(1, 2, poseOdometry, noiseOdometery);
    }

    void add_measurements_2(int camera_id, const std::vector<PoseWithCovariance> &camera_f_markers)
    {
      // Have to see at least two markers before this routine can do anything.
      if (camera_f_markers.size() < 2) {
        return;
      }

      // Start with a fresh graph.
      graph_.resize(0);
      initialEstimate_.clear();

      // List of markers that we have never seen.
      std::vector<const PoseWithCovariance *> unknown_camera_f_markers{};

      // The camera pose initial value is figured from one of the known markers
      bool camera_f_world_inited = false;
      gtsam::Pose3 camera_f_world_estimate{};

      // Loop through the measurements adding priors for any markers we have already located to some degree
      for (auto &camera_f_marker : camera_f_markers) {

        const gtsam::Pose3 *known_marker_f_world{nullptr};
        gtsam::SharedNoiseModel known_noise_model{};

        // If this is the key marker, then add the prior
        if (camera_f_marker.id_ == key_marker_id_) {
          static auto key_prior = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << gtsam::Vector3::Constant(1e-5), gtsam::Vector3::Constant(1e-5))
              .finished());
          known_noise_model = key_prior;
          known_marker_f_world = &key_marker_f_world_;

        } else {
          // See if this measurement is for a known marker.
          auto marker_it = markers_known_.find(camera_f_marker.id_);
          if (marker_it != markers_known_.end()) {
            auto cov = marker_it->second.cov_; // :( DOn't know why segfault happens if not copied
            known_noise_model = gtsam::noiseModel::Gaussian::Covariance(cov);
            known_marker_f_world = &marker_it->second.pose_;
          }
        }

        // If no known marker has the id of the measurement, then there is no prior to add. Save
        // this measurement so it can be added to the graph later.
        if (known_marker_f_world == nullptr) {
          unknown_camera_f_markers.emplace_back(&camera_f_marker);
          continue;
        }

        // Add the prior for the known marker_f_world.
        graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('m', camera_f_marker.id_),
                                                                 *known_marker_f_world,
                                                                 known_noise_model);

        // Add the initial estimate for the known marker_f_world
        initialEstimate_.insert(gtsam::Symbol('m', camera_f_marker.id_), *known_marker_f_world);

        // Add the measurment between the known marker and the camera
        auto cov = camera_f_marker.cov_;  // :( Not quite sure why I have to make a copy here. get seg_fault otherwise
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('m', camera_f_marker.id_),
          gtsam::Symbol('c', camera_id),
          camera_f_marker.pose_,
          gtsam::noiseModel::Gaussian::Covariance(cov));

        // Add the initialEstimate for the camera if it has not been added already.
        if (!camera_f_world_inited) {
          camera_f_world_inited = true;
          camera_f_world_estimate = *known_marker_f_world * camera_f_marker.pose_;
          initialEstimate_.insert(gtsam::Symbol('c', camera_id), camera_f_world_estimate);
        }
      }

      // If there were no known markers in the measurements, then just exit as there is nothing to do.
      if (!camera_f_world_inited) {
        return;
      }

      // Add the measurements to unknown markers to the graph. Use the estimate of the camera pose
      // to calculate initial estimates for these marker poses
      for (auto unknown_camera_f_marker : unknown_camera_f_markers) {

        // Add the measurment between the unknown marker and the camera
        auto cov = unknown_camera_f_marker->cov_;// :( same as above
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('m', unknown_camera_f_marker->id_),
          gtsam::Symbol('c', camera_id),
          unknown_camera_f_marker->pose_,
          gtsam::noiseModel::Gaussian::Covariance(cov));

        // derive an estimate of the marker pose from the camera pose estimate
        auto marker_f_world = camera_f_world_estimate * unknown_camera_f_marker->pose_.inverse();
        initialEstimate_.insert(gtsam::Symbol('m', unknown_camera_f_marker->id_), marker_f_world);
      }

      std::cout << std::endl
                << "Optimize for camera " << camera_id << std::endl;

      // Now optimize this graph
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initialEstimate_).optimize();
      std::cout << "initial error = " << graph_.error(initialEstimate_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      gtsam::Marginals marginals(graph_, result);

      // Print results
      const auto c_pose{result.at<gtsam::Pose3>(gtsam::Symbol('c', camera_id))};
      const gtsam::Matrix6 c_cov{marginals.marginalCovariance(gtsam::Symbol('c', camera_id))};
      std::cout << "c" << camera_id << std::endl
                << PoseWithCovariance::to_str(c_pose)
                << PoseWithCovariance::to_str(c_cov) << std::endl;
      for (auto &camera_f_marker : camera_f_markers) {
        const auto m_pose{result.at<gtsam::Pose3>(gtsam::Symbol('m', camera_f_marker.id_))};
        const gtsam::Matrix6 m_cov{marginals.marginalCovariance(gtsam::Symbol('m', camera_f_marker.id_))};
        std::cout << "m" << camera_f_marker.id_ << std::endl
                  << PoseWithCovariance::to_str(m_pose)
                  << PoseWithCovariance::to_str(m_cov) << std::endl;
      }

      // Save marker poses in the known marker array.
      for (auto &camera_f_marker : camera_f_markers) {

        // skip the key marker
        if (camera_f_marker.id_ == key_marker_id_) {
          continue;
        }

        // erase a known marker so it can be inserted again
        auto key = gtsam::Symbol('m', camera_f_marker.id_);
        auto marker_it = markers_known_.find(camera_f_marker.id_);
        if (marker_it != markers_known_.end()) {
          markers_known_.erase(marker_it);
        }

        // Add the new marker pose
        markers_known_.emplace(camera_f_marker.id_,
                               PoseWithCovariance{camera_f_marker.id_,
                                                  result.at<gtsam::Pose3>(key),
                                                  marginals.marginalCovariance(key)});
      }
    }
  };

  SfmIsam2::SfmIsam2(int key_marker_id, const gtsam::Pose3 &key_marker_f_world) :
    impl_{std::make_unique<SfmIsam2Impl>(key_marker_id, key_marker_f_world)}
  {}

  SfmIsam2::~SfmIsam2() = default;

  void SfmIsam2::add_measurements(
    int camera_id, const std::vector<PoseWithCovariance> &camera_f_markers)
  {
    impl_->add_measurements_2(camera_id, camera_f_markers);
  }
}

#include "model.hpp"

std::vector<camsim::PoseWithCovariance> sfm_run_isam2_camera_f_markers(
  camsim::Model &model,
  const gtsam::SharedNoiseModel &measurement_noise,
  const camsim::CameraModel &camera)
{
  std::vector<camsim::PoseWithCovariance> camera_f_markers{};

  camsim::CalcCameraPose ccp{model.cameras_.calibration_,
                             model.cameras_.project_func_,
                             measurement_noise,
                             model.markers_.corners_f_marker_};

  for (auto &marker : model.markers_.markers_) {

    auto &corners_f_image = model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;

    // If the marker was not visible in the image then, obviously, a pose calculation can not be done.
    if (corners_f_image.empty()) {
      std::cout << "Marker not visible" << std::endl << std::endl;
      continue;
    }

    // Find the camera pose in the marker frame using the GTSAM library and add it to the list
    camera_f_markers.emplace_back(ccp.camera_f_marker(marker.marker_idx_, corners_f_image));

    const auto &camera_f_marker = camera_f_markers.back();

    // Test that the calculated pose is the same as the original model.
    if (!camera_f_marker.pose_.equals(
      marker.marker_f_world_.inverse() * camera.camera_f_world_)) {
      std::cout << "calculated pose does not match the ground truth" << std::endl;
    }

    // Output the resulting pose and covariance
    std::cout << camera_f_marker.to_str() << std::endl;
  }

  return camera_f_markers;
}

int sfm_run_isam2()
{
  camsim::Model model{camsim::MarkersConfigurations::along_x_axis,
                      camsim::CamerasConfigurations::c_along_x_axis,
                      camsim::CameraTypes::distorted_camera};

  gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.5, 0.5));

  camsim::SfmIsam2 sfm_isam2{0, model.markers_.markers_[0].marker_f_world_};

  for (auto &camera : model.cameras_.cameras_) {
    std::cout << std::endl
              << "************************" << std::endl
              << "camera " << camera.camera_idx_ << " measurements of camera_f_marker" << std::endl;

    sfm_isam2.add_measurements(camera.camera_idx_,
                               sfm_run_isam2_camera_f_markers(model, measurement_noise, camera));
  }

  return EXIT_SUCCESS;
}
