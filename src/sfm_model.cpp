
#include "sfm_model.hpp"

namespace camsim
{

  // World Coordinate system: North Weest Up
  // Marker Coordinate system: Top Left Out when looking at Marker

  static std::vector<MarkerModel> gen_markers(MarkersConfigurations marker_configuration,
                                              double marker_size,
                                              const std::vector<gtsam::Point3> &corners_f_marker)
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

    std::vector<MarkerModel> markers{};
    for (std::size_t idx = 0; idx < marker_f_worlds.size(); idx += 1) {
      auto &marker_f_world = marker_f_worlds[idx];

      std::vector<gtsam::Point3> corners_f_world{};
      for (auto &corner_f_marker : corners_f_marker) {
        corners_f_world.emplace_back(marker_f_world * corner_f_marker);
      }

      markers.emplace_back(MarkerModel{idx, marker_f_world, std::move(corners_f_world)});
    }

    return markers;
  }

  MarkersModel::MarkersModel(MarkersConfigurations markers_configuration) :
    markers_configuration_{markers_configuration},
    marker_size_{20.0},
    corners_f_marker_{gtsam::Point3{-marker_size_ / 2, marker_size_ / 2, 0},
                      gtsam::Point3{marker_size_ / 2, marker_size_ / 2, 0},
                      gtsam::Point3{marker_size_ / 2, -marker_size_ / 2, 0},
                      gtsam::Point3{-marker_size_ / 2, -marker_size_ / 2, 0}},
    markers_{gen_markers(markers_configuration, marker_size_, corners_f_marker_)}
  {
    std::cout << "corners_f_worlds" << std::endl;
    for (auto &marker : markers_) {
      auto &corner_f_world = marker.corners_f_world_;
      std::cout << corner_f_world[0]
                << corner_f_world[1]
                << corner_f_world[2]
                << corner_f_world[3] << std::endl;
    }
  }


  static std::vector<CameraModel> gen_cameras(CamerasConfigurations camera_configuration,
                                              double marker_size,
                                              const gtsam::Cal3_S2 &camera_calibration)
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

    std::vector<CameraModel> cameras{};
    for (std::size_t idx = 0; idx < camera_f_worlds.size(); idx += 1) {
      auto &camera_f_world = camera_f_worlds[idx];

      cameras.emplace_back(CameraModel{idx, camera_f_world,
                                       gtsam::SimpleCamera{camera_f_world, camera_calibration}});
    }

    return cameras;
  }

  CamerasModel::CamerasModel(CamerasConfigurations cameras_configuration,
                             double marker_size) :
    cameras_configuration_{cameras_configuration},
    calibration_{1, 1, 0, 50, 50},
    cameras_{gen_cameras(cameras_configuration, marker_size, calibration_)}
  {}

  static std::vector<std::vector<CornersFImageModel>> gen_corners_f_images(const MarkersModel &markers,
                                                                           const CamerasModel &cameras)
  {
    std::vector<std::vector<CornersFImageModel>> corners_f_images;

    for (auto &camera : cameras.cameras_) {
      std::vector<CornersFImageModel> per_camera;
      for (auto &marker : markers.markers_) {
        per_camera.emplace_back(CornersFImageModel{
          marker.marker_idx_, camera.camera_idx_,
          std::vector<gtsam::Point2>{camera.simple_camera_.project(marker.corners_f_world_[0]),
                                     camera.simple_camera_.project(marker.corners_f_world_[1]),
                                     camera.simple_camera_.project(marker.corners_f_world_[2]),
                                     camera.simple_camera_.project(marker.corners_f_world_[3])}});
      }
      corners_f_images.emplace_back(per_camera);
    }
    return corners_f_images;
  }

  SfmModel::SfmModel(MarkersConfigurations markers_configuration,
                     CamerasConfigurations cameras_configuration) :
    markers_{markers_configuration},
    cameras_{cameras_configuration, markers_.marker_size_},
    corners_f_images_{gen_corners_f_images(markers_, cameras_)}
  {
    std::cout << "corners_f_images" << std::endl;
    for (auto &per_camera : corners_f_images_) {
      std::cout << "camera xx" << std::endl;
      for (auto &per_marker : per_camera) {
        auto &corners_f_image = per_marker.corners_f_image_;
        std::cout << corners_f_image[0]
                  << corners_f_image[1]
                  << corners_f_image[2]
                  << corners_f_image[3] << std::endl;
      }
    }
  }
}