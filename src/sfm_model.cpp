
#include "sfm_model.hpp"

namespace camsim
{

  // World Coordinate system: North Weest Up
  // Marker Coordinate system: Top Left Out when looking at Marker

  static std::vector<gtsam::Pose3> gen_marker_f_worlds(MarkersConfigurations marker_configuration,
                                                       double marker_size)
  {
    std::vector<gtsam::Pose3> marker_f_worlds{};

    if (marker_configuration == MarkersConfigurations::square_around_origin_xy_plane) {
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(marker_size, marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(marker_size, -marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(-marker_size, -marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(-marker_size, marker_size, 0)});
    }

    return marker_f_worlds;
  }

  static std::vector<std::vector<gtsam::Point3>> gen_corners_f_worlds(const std::vector<gtsam::Pose3> &marker_f_worlds,
                                                                      const std::vector<gtsam::Point3> corners_f_marker)
  {
    std::vector<std::vector<gtsam::Point3>> corners_f_worlds{};
    for (auto &marker_f_world : marker_f_worlds) {
      std::vector<gtsam::Point3> corners_f_world{};
      for (auto &corner_f_marker : corners_f_marker) {
        corners_f_world.emplace_back(marker_f_world * corner_f_marker);
      }
      corners_f_worlds.emplace_back(corners_f_world);
    }
    return corners_f_worlds;
  }

  MarkersModel::MarkersModel(MarkersConfigurations markers_configuration) :
    markers_configuration_{markers_configuration},
    marker_size_{20.0},
    corners_f_marker_{gtsam::Point3{-marker_size_ / 2, marker_size_ / 2, 0},
                      gtsam::Point3{marker_size_ / 2, marker_size_ / 2, 0},
                      gtsam::Point3{marker_size_ / 2, -marker_size_ / 2, 0},
                      gtsam::Point3{-marker_size_ / 2, -marker_size_ / 2, 0}},
    pose_f_worlds_{gen_marker_f_worlds(markers_configuration, marker_size_)},
    corners_f_worlds_{gen_corners_f_worlds(pose_f_worlds_, corners_f_marker_)}
  {
    std::cout << "corners_f_worlds" << std::endl;
    for (auto per_marker : corners_f_worlds_) {
      std::cout << per_marker[0] << per_marker[1] << per_marker[2] << per_marker[3] << std::endl;
    }
  }


  static std::vector<gtsam::Pose3> gen_camera_f_worlds(CamerasConfigurations camera_configuration,
                                                       double marker_size)
  {
    std::vector<gtsam::Pose3> camera_f_worlds{};

    if (camera_configuration == CamerasConfigurations::center_facing_markers) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                gtsam::Point3(0, 0, 2)});

    } else if (camera_configuration == CamerasConfigurations::square_around_z_axis) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                gtsam::Point3(marker_size, marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                gtsam::Point3(marker_size, -marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                gtsam::Point3(-marker_size, -marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                gtsam::Point3(-marker_size, marker_size, 2)});

    } else if (camera_configuration == CamerasConfigurations::fly_to_plus_y) {
      for (int i = -50; i <= 50; i += 1) {
        camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                                  gtsam::Point3(0, i * marker_size / 10, 2)});
      }
    }

    return camera_f_worlds;
  }

  static std::vector<gtsam::SimpleCamera> gen_cameras(const gtsam::Cal3_S2 &camera_calibration,
                                                      const std::vector<gtsam::Pose3> &camera_f_worlds)
  {
    std::vector<gtsam::SimpleCamera> cameras;
    for (auto &camera_f_world : camera_f_worlds) {
      cameras.emplace_back(gtsam::SimpleCamera{camera_f_world, camera_calibration});
    }
    return cameras;
  }

  static std::vector<std::vector<std::vector<gtsam::Point2>>> gen_corners_f_images(
    const std::vector<gtsam::SimpleCamera> &cameras,
    const std::vector<std::vector<gtsam::Point3>> &corners_f_worlds)
  {
    std::vector<std::vector<std::vector<gtsam::Point2>>> corners_f_images;
    for (auto &camera : cameras) {
      std::vector<std::vector<gtsam::Point2>> per_camera;
      for (auto &corners_f_world : corners_f_worlds) {
        per_camera.emplace_back(std::vector<gtsam::Point2>{camera.project(corners_f_world[0]),
                                                           camera.project(corners_f_world[1]),
                                                           camera.project(corners_f_world[2]),
                                                           camera.project(corners_f_world[3])});
      }
      corners_f_images.emplace_back(per_camera);
    }
    return corners_f_images;
  }

  CamerasModel::CamerasModel(CamerasConfigurations cameras_configuration,
                             double marker_size,
                             const std::vector<std::vector<gtsam::Point3>> &corners_f_worlds) :
    cameras_configuration_{cameras_configuration},
    calibration_{1, 1, 0, 50, 50},
    pose_f_worlds_{gen_camera_f_worlds(cameras_configuration, marker_size)},
    cameras_{gen_cameras(calibration_, pose_f_worlds_)},
    corners_f_images_{gen_corners_f_images(cameras_, corners_f_worlds)}
  {
    std::cout << "corners_f_images" << std::endl;
    for (auto &per_camera : corners_f_images_) {
      std::cout << "camera xx" << std::endl;
      for (auto &per_marker : per_camera) {
        std::cout << per_marker[0] << per_marker[1] << per_marker[2] << per_marker[3] << std::endl;
      }
    }
  }

}