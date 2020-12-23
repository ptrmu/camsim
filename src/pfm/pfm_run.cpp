
#include "pfm_run.hpp"


#include <gtsam/geometry/Cal3_S2.h>
#include "../model.hpp"
#include "../pose_with_covariance.hpp"

namespace camsim
{

  static void compare_results(const gtsam::Cal3_S2 &camera_calibration,
                              const gtsam::SharedNoiseModel &measurement_noise,
                              const CameraModel &camera, const MarkerModel &marker,
                              const std::vector<gtsam::Point2> &corners_f_image)
  {
    std::cout << "camera_" << camera.index() << PoseWithCovariance::to_str(camera.camera_f_world_) << std::endl;
    std::cout << "marker_" << marker.index() << PoseWithCovariance::to_str(marker.marker_f_world_) << std::endl;

    // Get gtsam results
    gtsam::Pose3 gtsam_camera_f_world;
    gtsam::Matrix6 gtsam_camera_f_world_covariance;

    pfm_resection_gtsam(camera_calibration,
                        corners_f_image,
                        marker.corners_f_world_,
                        camera.camera_f_world_,
                        measurement_noise,
                        gtsam_camera_f_world, gtsam_camera_f_world_covariance);

    std::cout << "gtsam_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(gtsam_camera_f_world) << std::endl
              << PoseWithCovariance::to_matrix_str(gtsam_camera_f_world_covariance, true) << std::endl;

    // Get projection results
    gtsam::Pose3 projection_camera_f_world;
    gtsam::Matrix6 projection_camera_f_world_covariance;

    pfm_resection_projection(camera_calibration,
                             corners_f_image,
                             marker.corners_f_world_,
                             camera.camera_f_world_,
                             measurement_noise,
                             projection_camera_f_world, projection_camera_f_world_covariance);

    std::cout << "projection_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(projection_camera_f_world) << std::endl
              << PoseWithCovariance::to_matrix_str(projection_camera_f_world_covariance, true) << std::endl;

    // Get opencv results
    gtsam::Pose3 opencv_camera_f_marker;
    gtsam::Matrix6 opencv_camera_f_marker_covariance;

    pfm_resection_opencv(camera_calibration,
                         corners_f_image,
                         marker.corners_f_world_,
                         camera.camera_f_world_,
                         measurement_noise,
                         opencv_camera_f_marker, opencv_camera_f_marker_covariance);

    std::cout << "opencv_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(opencv_camera_f_marker) << std::endl
              << PoseWithCovariance::to_matrix_str(opencv_camera_f_marker_covariance, true) << std::endl;

    // Get monte carlo results
    gtsam::Pose3 monte_carlo_camera_f_marker;
    gtsam::Matrix6 monte_carlo_camera_f_marker_covariance;

    pfm_resection_monte_carlo(camera_calibration,
                              corners_f_image,
                              marker.corners_f_world_,
                              camera.camera_f_world_,
                              measurement_noise,
                              monte_carlo_camera_f_marker, monte_carlo_camera_f_marker_covariance);

    std::cout << "monte_carlo_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(monte_carlo_camera_f_marker) << std::endl
              << PoseWithCovariance::to_matrix_str(monte_carlo_camera_f_marker_covariance, true) << std::endl;

    std::cout << std::endl;
  }

  static void pfm_run_multi(Model &model)
  {
    const gtsam::Cal3_S2 camera_calibration{model.cameras_.get_Cal3_S2()};
    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(.5, .5));

    for (auto &camera : model.cameras_.cameras_) {
      for (auto &marker : model.markers_.markers_) {
        auto &corners_f_image = model.corners_f_images_[camera.index()][marker.index()].corners_f_image_;
        if (!corners_f_image.empty()) {
          compare_results(camera_calibration, measurement_noise,
                          camera, marker, corners_f_image);
        }
      }
    }
  }

  int pfm_run(int model_id)
  {
    if (model_id == 0) {
      Model model{ModelConfig{MarkersConfigurations::single_center,
                              CamerasConfigurations::fly_to_plus_y,
                              CameraTypes::simple_camera}};

      pfm_run_multi(model);
    }

    if (model_id == 1) {
      Model model{ModelConfig{MarkersConfigurations::single_south_west,
                              CamerasConfigurations::far_south,
                              CameraTypes::simple_camera}};

      pfm_run_multi(model);
    }

    return EXIT_SUCCESS;
  }
}
