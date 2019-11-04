
#include "sfm_isam2.hpp"

namespace camsim
{

  void SfmIsam2::add_measurements(
    int camera_id, const std::vector<SfmPoseWithCovariance> &camera_f_markers)
  {

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

  camsim::SfmIsam2 sfm_isam2{};

  for (auto &camera : sfm_model.cameras_.cameras_) {
    std::cout << "camera " << camera.camera_idx_ << std::endl;

    auto camera_f_markers = sfm_run_isam2_camera_f_markers(sfm_model, measurement_noise, camera);
  }

  return EXIT_SUCCESS;
}
