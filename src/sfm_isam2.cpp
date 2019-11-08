
#include "sfm_isam2.hpp"

#include <set>
#include <map>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
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

    std::map<int, SfmPoseWithCovariance> markers_known_{};
    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initialEstimate_{};

  public:
    SfmIsam2Impl(int key_marker_id, const gtsam::Pose3 &key_marker_f_world) :
      key_marker_id_{key_marker_id}, key_marker_f_world_{key_marker_f_world}
    {
    }

    void add_measurements(int camera_id, const std::vector<SfmPoseWithCovariance> &camera_f_markers)
    {
//      graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(1, 2, poseOdometry, noiseOdometery);
    }

    void add_measurements_2(int camera_id, const std::vector<SfmPoseWithCovariance> &camera_f_markers)
    {
      // Have to see at least two markers before this routine can do anything.
      if (camera_f_markers.size() < 2) {
        return;
      }

      // Start with a fresh graph.
      graph_.resize(0);
      initialEstimate_.clear();
      bool camera_initial_added = false;

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
            known_noise_model = gtsam::noiseModel::Gaussian::Covariance(marker_it->second.cov_);
            known_marker_f_world = &marker_it->second.pose_;
          }
        }

        // If no known marker has the id of the measurement, then there is no prior to add.
        if (known_marker_f_world == nullptr) {
          continue;
        }

        // Add the prior for the known marker_f_world.
        graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('m', camera_f_marker.id_),
                                                                 *known_marker_f_world,
                                                                 known_noise_model);

        // Add the initial estimate for the known marker_f_world
        initialEstimate_.insert(gtsam::Symbol('m', camera_f_marker.id_), *known_marker_f_world);

        // Add the measurment between the known marker and the camera
       graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          gtsam::Symbol('c', camera_id),
          gtsam::Symbol('m', camera_f_marker.id_),
          camera_f_marker.pose_,
          gtsam::noiseModel::Gaussian::Covariance(camera_f_marker.cov_));

        // Add the initialEstimate for the camera if it has not been added already.
        if (!camera_initial_added) {
          camera_initial_added = true;
          initialEstimate_.insert(gtsam::Symbol('c', camera_id), *known_marker_f_world * camera_f_marker.pose_);
        }
      }
    }
  };

  SfmIsam2::SfmIsam2(int key_marker_id, const gtsam::Pose3 &key_marker_f_world) :
    impl_{std::make_unique<SfmIsam2Impl>(key_marker_id, key_marker_f_world)}
  {}

  SfmIsam2::~SfmIsam2() = default;

  void SfmIsam2::add_measurements(
    int camera_id, const std::vector<SfmPoseWithCovariance> &camera_f_markers)
  {
    impl_->add_measurements(camera_id, camera_f_markers);
  }
}

#include "sfm_model.hpp"

std::vector<camsim::SfmPoseWithCovariance> sfm_run_isam2_camera_f_markers(
  camsim::SfmModel &sfm_model,
  const gtsam::SharedNoiseModel &measurement_noise,
  const camsim::CameraModel &camera)
{
  std::vector<camsim::SfmPoseWithCovariance> camera_f_markers{};

  camsim::CalcCameraPose ccp{sfm_model.cameras_.calibration_,
                             measurement_noise,
                             sfm_model.markers_.corners_f_marker_};

  for (auto &marker : sfm_model.markers_.markers_) {

    auto &corners_f_image = sfm_model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;

    // If the marker was not visible in the image then, obviously, a pose calculation can not be done.
    if (corners_f_image.empty()) {
      std::cout << "Marker not visible" << std::endl;
      continue;
    }

    // Find the camera pose in the marker frame using the GTSAM library and add it to the list
    camera_f_markers.emplace_back(ccp.camera_f_marker(marker.marker_idx_, corners_f_image));

    const auto &camera_f_marker = camera_f_markers.back();

    // Test that the calculated pose is the same as the original model.
    if (!camera_f_marker.pose_.equals(
      marker.pose_f_world_.inverse() * camera.pose_f_world_)) {
      std::cout << "calculated pose does not match the ground truth" << std::endl;
    }

    // Output the resulting pose and covariance
    std::cout << sfm_model.to_str(camera_f_marker) << std::endl;
  }

  return camera_f_markers;
}

int sfm_run_isam2()
{
  camsim::SfmModel sfm_model{camsim::MarkersConfigurations::square_around_origin_xy_plane,
                             camsim::CamerasConfigurations::fly_to_plus_y};

  gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.5, 0.5));

  camsim::SfmIsam2 sfm_isam2{0, sfm_model.markers_.markers_[0].pose_f_world_};

  for (auto &camera : sfm_model.cameras_.cameras_) {
    std::cout << "camera " << camera.camera_idx_ << std::endl;

    sfm_isam2.add_measurements(camera.camera_idx_,
                               sfm_run_isam2_camera_f_markers(sfm_model, measurement_noise, camera));
  }

  return EXIT_SUCCESS;
}
