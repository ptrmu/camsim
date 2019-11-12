
#include "model.hpp"
#include "sfm_pose_with_covariance.hpp"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <iomanip>

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

    } else if (marker_configuration == MarkersConfigurations::along_x_axis) {
      int marker_number = 3;
      double marker_spacing = 5 * marker_size;

      for (int i = 0; i < marker_number; i += 1) {
        double offset = i - static_cast<double>(marker_number - 1) / 2;
        marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                  gtsam::Point3(offset * marker_spacing, 0, 0)});
      }
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

  static gtsam::Cal3DS2 gen_camera_calibration(CameraTypes camera_type)
  {
    return gtsam::Cal3DS2{1, 1, 0, 50, 50, 0., 0.};
  }

  static gtsam::Point2 project_point(CameraTypes camera_type,
                                     const gtsam::Cal3DS2 &calibration,
                                     const gtsam::Pose3 &pose,
                                     const gtsam::Point3 &pw,
                                     boost::optional<gtsam::Matrix &> H)
  {
    if (camera_type == CameraTypes::simple_camera) {
      gtsam::Cal3_S2 K{calibration.fx(), calibration.fy(),
                       calibration.skew(),
                       calibration.px(), calibration.py()};
      auto camera = gtsam::SimpleCamera{pose, K};
      return camera.project(pw, H);
    }

    if (camera_type == CameraTypes::distorted_camera) {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, calibration};
      return camera.project(pw, H);
    }

    return gtsam::Point2{};
  }

  static std::vector<CameraModel> gen_cameras(CameraTypes camera_type,
                                              CamerasConfigurations camera_configuration,
                                              const gtsam::Cal3DS2 &calibration,
                                              double marker_size)
  {
    std::vector<gtsam::Pose3> camera_f_worlds{};

    if (camera_configuration == CamerasConfigurations::center_facing_markers) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(0, 0, 2)});

    } else if (camera_configuration == CamerasConfigurations::plus_x_facing_markers) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(marker_size, 0, 2)});

    } else if (camera_configuration == CamerasConfigurations::square_around_z_axis) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(marker_size, marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(marker_size, -marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(-marker_size, -marker_size, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(-marker_size, marker_size, 2)});

    } else if (camera_configuration == CamerasConfigurations::fly_to_plus_y) {
      for (int i = -10; i <= 10; i += 1) {
        camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                  gtsam::Point3(0, i * marker_size / 2, 2)});
      }

    } else if (camera_configuration == CamerasConfigurations::c_along_x_axis) {
      int camera_number = 5;
      double camera_spacing = 1.25 * marker_size;

      for (int i = 0; i < camera_number; i += 1) {
        double offset = i - static_cast<double>(camera_number - 1) / 2;
        camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                  gtsam::Point3(offset * camera_spacing, 0, 2)});
      }
    }

    std::vector<CameraModel> cameras{};
    for (std::size_t idx = 0; idx < camera_f_worlds.size(); idx += 1) {
      auto &camera_f_world = camera_f_worlds[idx];

      cameras.emplace_back(CameraModel{
        idx,
        camera_f_world,
        [camera_type, calibration](const gtsam::Pose3 &camera_f_xxx,
                                   const gtsam::Point3 &point_f_xxx,
                                   boost::optional<gtsam::Matrix &> H) -> gtsam::Point2
        { return project_point(camera_type, calibration, camera_f_xxx, point_f_xxx, H); }});
    }

    return cameras;
  }

  CamerasModel::CamerasModel(CameraTypes camera_type,
                             CamerasConfigurations cameras_configuration,
                             double marker_size) :
    camera_type_{camera_type},
    cameras_configuration_{cameras_configuration},
    calibration_{gen_camera_calibration(camera_type)},
    cameras_{gen_cameras(camera_type, cameras_configuration, calibration_, marker_size)}
  {}

  static std::vector<std::vector<CornersFImageModel>> gen_corners_f_images(const MarkersModel &markers,
                                                                           const CamerasModel &cameras)
  {
    std::vector<std::vector<CornersFImageModel>> corners_f_images;

    for (auto &camera : cameras.cameras_) {
      std::vector<CornersFImageModel> per_camera;
      for (auto &marker : markers.markers_) {
        // Project the corners of this marker into this camera's image plane. There should be more
        // checks for bad geometries before calling project because it aborts in those cases.
        std::vector<gtsam::Point2> corners_f_image{
          camera.project_func_(camera.pose_f_world_, marker.corners_f_world_[0], boost::none),
          camera.project_func_(camera.pose_f_world_, marker.corners_f_world_[1], boost::none),
          camera.project_func_(camera.pose_f_world_, marker.corners_f_world_[2], boost::none),
          camera.project_func_(camera.pose_f_world_, marker.corners_f_world_[3], boost::none)};

        // If any of the points is outside of the image boundary, then don't save any of the points. This
        // simulates when a marker can't be seen by a camera.
        for (auto &corner_f_image : corners_f_image) {
          if (corner_f_image.x() < 0 || corner_f_image.x() >= 2 * cameras.calibration_.px() ||
              corner_f_image.y() < 0 || corner_f_image.y() >= 2 * cameras.calibration_.py()) {
            corners_f_image.clear();
            break;
          }
        }

        // Add an entry for this marker's corners in the camera's array
        per_camera.emplace_back(CornersFImageModel{marker.marker_idx_, camera.camera_idx_, corners_f_image});
      }

      corners_f_images.emplace_back(per_camera);
    }
    return corners_f_images;
  }

  Model::Model(MarkersConfigurations markers_configuration,
               CamerasConfigurations cameras_configuration,
               CameraTypes camera_type) :
    markers_{markers_configuration},
    cameras_{camera_type, cameras_configuration, markers_.marker_size_},
    corners_f_images_{gen_corners_f_images(markers_, cameras_)}
  {
    std::cout << "corners_f_images" << std::endl;
    for (auto &per_camera : corners_f_images_) {
      for (auto &per_marker : per_camera) {

        if (per_marker.marker_idx_ == 0) {
          std::cout << "camera " << per_marker.camera_idx_ << std::endl;
        }

        auto &corners_f_image = per_marker.corners_f_image_;

        // If the array is empty, then the marker was not visible to the camera.
        if (corners_f_image.empty()) {
          std::cout << "  Marker not visible" << std::endl;
        } else {
          std::cout << "  "
                    << corners_f_image[0]
                    << corners_f_image[1]
                    << corners_f_image[2]
                    << corners_f_image[3] << std::endl;
        }
      }
    }
  }
}
