
#include "pfm_run.hpp"


#include <gtsam/geometry/Cal3_S2.h>
#include "model.hpp"
#include "pfm_model.hpp"
#include "pose_with_covariance.hpp"

namespace camsim
{

  enum ModelTypes
  {
    like_gtsam_resection = 0,
    shift_camera_in_z,
    shift_marker_in_z,
    shift_marker_in_x,
    shift_marker_in_y,
    small_rotate_camera__about_x,
    small_rotate_camera__about_y,
  };

  std::unique_ptr<PfmModel> get_model(ModelTypes model_type)
  {
    gtsam::Cal3_S2 camera_calibration{1, 1, 0, 50, 50};

    gtsam::Pose3 camera_f_world{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                gtsam::Point3(0, 0, 2)};
    gtsam::Pose3 marker_f_world{gtsam::Rot3::RzRyRx(0, 0, 0),
                                gtsam::Point3(0, 0, 0)};

    double marker_size = 20.0;

    switch (model_type) {
      case ModelTypes::like_gtsam_resection:
        break;

      case ModelTypes::shift_camera_in_z:
        camera_f_world = gtsam::Pose3{camera_f_world.rotation(),
                                      camera_f_world.translation() + gtsam::Point3{0, 0, 1}};
        break;

      case ModelTypes::shift_marker_in_z:
        marker_f_world = gtsam::Pose3{marker_f_world.rotation(),
                                      marker_f_world.translation() + gtsam::Point3{0, 0, -0.5}};
        break;

      case ModelTypes::shift_marker_in_x:
        marker_f_world = gtsam::Pose3{marker_f_world.rotation(),
                                      marker_f_world.translation() + gtsam::Point3{6, 0, 0}};
        break;

      case ModelTypes::shift_marker_in_y:
        marker_f_world = gtsam::Pose3{marker_f_world.rotation(),
                                      marker_f_world.translation() + gtsam::Point3{0, 6, 0}};
        break;

      case ModelTypes::small_rotate_camera__about_x:
        camera_f_world = gtsam::Pose3{camera_f_world.rotation() * gtsam::Rot3::Rx(0.05),
                                      camera_f_world.translation()};
        break;

      case ModelTypes::small_rotate_camera__about_y:
        camera_f_world = gtsam::Pose3{camera_f_world.rotation() * gtsam::Rot3::Ry(0.05),
                                      camera_f_world.translation()};
        break;
    }

    return std::make_unique<PfmModel>(camera_calibration,
                                      camera_f_world,
                                      marker_f_world,
                                      marker_size);
  }

  static void compare_results(const gtsam::Cal3_S2 &camera_calibration,
                              const gtsam::SharedNoiseModel &measurement_noise,
                              const CameraModel &camera, const MarkerModel &marker,
                              const std::vector<gtsam::Point2> &corners_f_image)
  {
    std::cout << "camera_" << camera.camera_idx_ << PoseWithCovariance::to_str(camera.pose_f_world_) << std::endl;
    std::cout << "marker_" << marker.marker_idx_ << PoseWithCovariance::to_str(marker.pose_f_world_) << std::endl;

    // Get gtsam results
    gtsam::Pose3 gtsam_camera_f_world;
    gtsam::Matrix6 gtsam_camera_f_world_covariance;

    pfm_resection_gtsam(camera_calibration,
                        corners_f_image,
                        marker.corners_f_world_,
                        camera.pose_f_world_,
                        measurement_noise,
                        gtsam_camera_f_world, gtsam_camera_f_world_covariance);

    std::cout << "gtsam_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(gtsam_camera_f_world) << std::endl
              << PoseWithCovariance::to_str(gtsam_camera_f_world_covariance) << std::endl;

    // Get projection results
    gtsam::Pose3 projection_camera_f_world;
    gtsam::Matrix6 projection_camera_f_world_covariance;

    pfm_resection_projection(camera_calibration,
                             corners_f_image,
                             marker.corners_f_world_,
                             camera.pose_f_world_,
                             measurement_noise,
                             projection_camera_f_world, projection_camera_f_world_covariance);

    std::cout << "projection_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(projection_camera_f_world) << std::endl
              << PoseWithCovariance::to_str(projection_camera_f_world_covariance) << std::endl;

    // Get opencv results
    gtsam::Pose3 opencv_camera_f_marker;
    gtsam::Matrix6 opencv_camera_f_marker_covariance;

    pfm_resection_opencv(camera_calibration,
                         corners_f_image,
                         marker.corners_f_world_,
                         camera.pose_f_world_,
                         measurement_noise,
                         opencv_camera_f_marker, opencv_camera_f_marker_covariance);

    std::cout << "opencv_camera_f_world" << std::endl
              << PoseWithCovariance::to_str(opencv_camera_f_marker) << std::endl
              << PoseWithCovariance::to_str(opencv_camera_f_marker_covariance) << std::endl;

    std::cout << std::endl;
  }

  static void pfm_run_multi(Model &model)
  {
    const gtsam::Cal3_S2 camera_calibration{model.cameras_.get_Cal3_S2()};
    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(.5, .5));

    for (auto &camera : model.cameras_.cameras_) {
      for (auto &marker : model.markers_.markers_) {
        auto &corners_f_image = model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;
        if (!corners_f_image.empty()) {
          compare_results(camera_calibration, measurement_noise,
                          camera, marker, corners_f_image);
        }
      }
    }
  }

  static int pfm_run()
  {
    Model model{MarkersConfigurations::single_center,
                CamerasConfigurations::fly_to_plus_y,
                CameraTypes::simple_camera};

//    Model model{MarkersConfigurations::single_south_west,
//                CamerasConfigurations::far_south,
//                CameraTypes::simple_camera};

    pfm_run_multi(model);

    return EXIT_SUCCESS;
  }
}

int main()
{
  return camsim::pfm_run();
//  return camsim::pfm_simple_rotation_example();
//  return camsim::pfm_optimize_pose3();
}
