
#include "model.hpp"
#include "pose_with_covariance.hpp"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/SimpleCamera.h>

#include <iomanip>

namespace camsim
{

  // World Coordinate system: East North Up
  // Marker Coordinate system: Right Up Out when looking at Marker

  std::vector<gtsam::Pose3> PoseGens::Noop::operator()() const
  {
    return std::vector<gtsam::Pose3>{};
  }

  static std::vector<gtsam::Pose3> rotate_around_z(int n, const gtsam::Pose3 &base)
  {
    std::vector<gtsam::Pose3> pose_f_worlds{};
    double delta_rotz = M_PI * 2 / n;
    for (int i = 0; i < n; i += 1) {
      auto rotz = delta_rotz * i;
      pose_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., rotz), gtsam::Point3{}} * base);
    }
    return pose_f_worlds;
  }

  std::vector<gtsam::Pose3> PoseGens::CircleInXYPlaneFacingOrigin::operator()() const
  {
    return rotate_around_z(n_, gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI_2, 0., -M_PI_2),
                                            gtsam::Point3{radius_, 0., 0.}});
  }

  std::vector<gtsam::Pose3> PoseGens::SpinAboutZAtOriginFacingOut::operator()() const
  {
    static gtsam::Pose3 base{gtsam::Rot3::RzRyRx(M_PI_2, 0., M_PI_2), gtsam::Point3{}};
    return rotate_around_z(n_, base);
  }

  static double gen_marker_size(MarkersConfigurations markers_configuration)
  {
    switch (markers_configuration) {
      default:
        return 20.0;
    }
  }

  static double gen_marker_spacing(MarkersConfigurations markers_configuration, double marker_size)
  {
    switch (markers_configuration) {
      default:
        return 2. * marker_size;
    }
  }

  static double gen_camera_spacing(CamerasConfigurations cameras_configuration, double marker_spacing)
  {
    switch (cameras_configuration) {
      default:
        return marker_spacing;
    }
  }

  ModelConfig::ModelConfig(MarkersConfigurations markers_configuration, CamerasConfigurations cameras_configuration,
                           CameraTypes camera_type) :
    markers_configuration_{markers_configuration},
    camera_type_{camera_type},
    cameras_configuration_{cameras_configuration},
    marker_size_{gen_marker_size(markers_configuration)},
    marker_spacing_{gen_marker_spacing(markers_configuration, marker_size_)},
    camera_spacing_{gen_camera_spacing(cameras_configuration, marker_spacing_)},
    marker_pose_generator_{PoseGens::Noop{}},
    camera_pose_generator_{PoseGens::Noop{}}
  {}

  ModelConfig::ModelConfig(PoseGenerator marker_pose_generator,
                           PoseGenerator camera_pose_generator,
                           CameraTypes camera_type,
                           double marker_size) :
    markers_configuration_{MarkersConfigurations::generator},
    camera_type_{camera_type},
    cameras_configuration_{CamerasConfigurations::generator},
    marker_size_{marker_size},
    marker_spacing_{0.},
    camera_spacing_{0.},
    marker_pose_generator_{std::move(marker_pose_generator)},
    camera_pose_generator_{std::move(camera_pose_generator)}
  {}

  ModelConfig::ModelConfig(const ModelConfig &model_config) :
    markers_configuration_{model_config.markers_configuration_},
    camera_type_{model_config.camera_type_},
    cameras_configuration_{model_config.cameras_configuration_},
    marker_size_{model_config.marker_size_},
    marker_spacing_{model_config.marker_spacing_},
    camera_spacing_{model_config.camera_spacing_},
    marker_pose_generator_{model_config.marker_pose_generator_},
    camera_pose_generator_{model_config.camera_pose_generator_}
  {}

  static void add_sphere_points(std::vector<gtsam::Point3> vectors,
                                double radius,
                                std::vector<gtsam::Pose3> &marker_f_worlds)
  {
    for (auto &vector : vectors) {
      auto n = vector.normalized();

      // Position of marker
      auto t = n * radius;

      // Rotation of the marker - facing the origin with the x axis horizontal.
      auto r = gtsam::Rot3::RzRyRx(0, 0, 0);
      auto d = gtsam::Point2{n.x(), n.y()}.norm();

      // If the normal is close to vertical
      if (d < 1e-4) {
        // The initialized r works for normal pointing up.
        // Flip r if the normal is pointing down.
        if (n.z() > 0.) {
          r = gtsam::Rot3::RzRyRx(M_PI, 0., 0.);
        }

      } else {
        // For non-vertical normals
        auto z_axis = -n;
        auto x_axis = gtsam::Point3{0., 0., 1.}.cross(z_axis).normalized();
        auto y_axis = z_axis.cross(x_axis);
        auto mat_r = (gtsam::Matrix3{} << x_axis, y_axis, z_axis).finished();
        r = gtsam::Rot3{mat_r};
      }

      marker_f_worlds.emplace_back(gtsam::Pose3{r, t});
    }
  }

  static std::vector<MarkerModel> gen_markers(const ModelConfig &cfg,
                                              const std::vector<gtsam::Point3> &corners_f_marker)
  {
    auto marker_size = cfg.marker_size_;
    auto markers_configuration = cfg.markers_configuration_;

    std::vector<gtsam::Pose3> marker_f_worlds{};

    if (markers_configuration == MarkersConfigurations::generator) {
      marker_f_worlds = cfg.marker_pose_generator_();

    } else if (markers_configuration == MarkersConfigurations::square_around_origin_xy_plane) {
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(marker_size, marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(marker_size, -marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(-marker_size, -marker_size, 0)});
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(-marker_size, marker_size, 0)});

    } else if (markers_configuration == MarkersConfigurations::single_center) {
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(0, 0, 0)});

    } else if (markers_configuration == MarkersConfigurations::single_south_west) {
      marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                gtsam::Point3(-marker_size, -marker_size, 0)});

    } else if (markers_configuration == MarkersConfigurations::along_x_axis) {
      int marker_number = 3;
      double marker_spacing = 5 * marker_size;

      for (int i = 0; i < marker_number; i += 1) {
        double offset = i - static_cast<double>(marker_number - 1) / 2;
        marker_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(0, 0, 0),
                                                  gtsam::Point3(offset * marker_spacing, 0, 0)});
      }

    } else if (markers_configuration == MarkersConfigurations::circle_around_z_axis) {
      int n = 8;
      double radius = 2. * marker_size;
      double delta_theta = M_PI * 2 / n;
      for (int i = 0; i < n; i += 1) {
        auto theta = delta_theta * i;
        marker_f_worlds.emplace_back(gtsam::Pose3{
          gtsam::Rot3::RzRyRx(0, 0, theta),
          gtsam::Point3(std::cos(theta) * radius, std::sin(theta) * radius, 0)});
      }

    } else if (markers_configuration == MarkersConfigurations::upright_circle_around_z_axis) {
      int n = 8;
      double radius = 2. * marker_size;
      double delta_theta = M_PI * 2 / n;
      for (int i = 0; i < n; i += 1) {
        auto theta = delta_theta * i;
        marker_f_worlds.emplace_back(gtsam::Pose3{
          gtsam::Rot3::Ypr(-M_PI_2 + theta, 0., M_PI_2),
          gtsam::Point3(std::cos(theta) * radius, std::sin(theta) * radius, 0)});
      }

    } else if (markers_configuration == MarkersConfigurations::tetrahedron) {
      add_sphere_points({gtsam::Point3{1, 1, 1},
                         gtsam::Point3{-1, -1, 1},
                         gtsam::Point3{1, -1, -1},
                         gtsam::Point3{-1, 1, -1},},
                        marker_size * 4, marker_f_worlds);

    } else if (markers_configuration == MarkersConfigurations::cube) {
      add_sphere_points({gtsam::Point3{1, 1, 1},
                         gtsam::Point3{-1, 1, 1},
                         gtsam::Point3{1, -1, 1},
                         gtsam::Point3{-1, -1, 1},
                         gtsam::Point3{1, 1, -1},
                         gtsam::Point3{-1, 1, -1},
                         gtsam::Point3{1, -1, -1},
                         gtsam::Point3{-1, -1, -1},},
                        marker_size * 4, marker_f_worlds);

    } else if (markers_configuration == MarkersConfigurations::octahedron) {
      add_sphere_points({gtsam::Point3{1, 0, 0},
                         gtsam::Point3{0, 1, 0},
                         gtsam::Point3{-1, 0, 0},
                         gtsam::Point3{0, -1, 0},
                         gtsam::Point3{0, 0, 1},
                         gtsam::Point3{0, 0, -1},},
                        marker_size * 4, marker_f_worlds);
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

  MarkersModel::MarkersModel(const ModelConfig &cfg) :
    cfg_{cfg},
    corners_f_marker_{gtsam::Point3{-cfg_.marker_size_ / 2, cfg_.marker_size_ / 2, 0},
                      gtsam::Point3{cfg_.marker_size_ / 2, cfg_.marker_size_ / 2, 0},
                      gtsam::Point3{cfg_.marker_size_ / 2, -cfg_.marker_size_ / 2, 0},
                      gtsam::Point3{-cfg_.marker_size_ / 2, -cfg_.marker_size_ / 2, 0}},
    markers_{gen_markers(cfg_, corners_f_marker_)}
  {
    std::cout << "Markers" << std::endl;
    for (auto &marker : markers_) {
      auto &corner_f_world = marker.corners_f_world_;
      std::cout << PoseWithCovariance::to_str(marker.marker_f_world_) << " ";
      std::cout << corner_f_world[0]
                << corner_f_world[1]
                << corner_f_world[2]
                << corner_f_world[3] << std::endl;
    }
  }

  MarkersModel::MarkersModel(const ModelConfig &cfg, const MarkersModel &copy) :
    cfg_{cfg},
    corners_f_marker_{copy.corners_f_marker_},
    markers_{copy.markers_}
  {}

  static gtsam::Cal3DS2 gen_camera_calibration(const ModelConfig &cfg_)
  {
    return gtsam::Cal3DS2{1, 1, 0, 50, 50, 0., 0.};
  }

  static std::vector<CameraModel> gen_cameras(const ModelConfig &cfg,
                                              const gtsam::Cal3DS2 &calibration)
  {
    std::vector<gtsam::Pose3> camera_f_worlds{};

    if (cfg.cameras_configuration_ == CamerasConfigurations::generator) {
      camera_f_worlds = cfg.camera_pose_generator_();
      static gtsam::Pose3 base{gtsam::Rot3::RzRyRx(0., 0., M_PI), gtsam::Point3{}};
      for (auto &camera_f_world : camera_f_worlds) {
        camera_f_world = camera_f_world * base;
      }

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::z2_facing_origin) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(0, 0, 2)});

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::center_looking_x) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(-M_PI_2, 0., -M_PI_2),
                                                gtsam::Point3(0., 0., 0.)});

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::far_south) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., 0.),
                                                gtsam::Point3(0., -100., 2.)});

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::plus_x_facing_markers) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(cfg.marker_size_, 0, 2)});

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::square_around_z_axis) {
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(cfg.marker_size_, cfg.marker_size_, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(cfg.marker_size_, -cfg.marker_size_, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(-cfg.marker_size_, -cfg.marker_size_, 2)});
      camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                gtsam::Point3(-cfg.marker_size_, cfg.marker_size_, 2)});

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::fly_to_plus_y) {
      const int camera_number = 5;
      const double min = -80.0;
      const double max = 80.0;
      const double delta = (max - min) / (camera_number - 1);

      for (int i = 0; i < camera_number; i += 1) {
        double offset = i * delta + min;
        camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., 0.),
                                                  gtsam::Point3(0., offset, 2.)});
      }

    } else if (cfg.cameras_configuration_ == CamerasConfigurations::c_along_x_axis) {
      int camera_number = 5;
      double camera_spacing = 1.25 * cfg.marker_size_;

      for (int i = 0; i < camera_number; i += 1) {
        double offset = i - static_cast<double>(camera_number - 1) / 2;
        camera_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0, 0),
                                                  gtsam::Point3(offset * camera_spacing, 0, 2)});
      }
    }

    std::vector<CameraModel> cameras{};
    for (std::size_t idx = 0; idx < camera_f_worlds.size(); idx += 1) {
      cameras.emplace_back(CameraModel{idx, camera_f_worlds[idx]});
    }

    return cameras;
  }

  static ProjectFunc gen_project_func(const ModelConfig &cfg,
                                      const gtsam::Cal3DS2 &calibration)
  {
    if (cfg.camera_type_ == CameraTypes::simple_camera) {
      gtsam::Cal3_S2 K{calibration.fx(), calibration.fy(),
                       calibration.skew(),
                       calibration.px(), calibration.py()};
      return [K](const gtsam::Pose3 &camera_f_xxx,
                 const gtsam::Point3 &point_f_xxx,
                 boost::optional<gtsam::Matrix &> H) -> gtsam::Point2
      {
        auto camera = gtsam::SimpleCamera{camera_f_xxx, K};
        return camera.project(point_f_xxx, H);
      };
    }

    if (cfg.camera_type_ == CameraTypes::distorted_camera) {
      return [calibration](const gtsam::Pose3 &camera_f_xxx,
                           const gtsam::Point3 &point_f_xxx,
                           boost::optional<gtsam::Matrix &> H) -> gtsam::Point2
      {
        auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_xxx, calibration};
        return camera.project(point_f_xxx, H);
      };
    }

    return [](const gtsam::Pose3 &camera_f_xxx,
              const gtsam::Point3 &point_f_xxx,
              boost::optional<gtsam::Matrix &> H) -> gtsam::Point2
    {
      return gtsam::Point2{};
    };
  }

  CamerasModel::CamerasModel(const ModelConfig &cfg) :
    cfg_{cfg},
    calibration_{gen_camera_calibration(cfg_)},
    project_func_{gen_project_func(cfg_, calibration_)},
    cameras_{gen_cameras(cfg, calibration_)}
  {
    std::cout << "Cameras" << std::endl;
    for (auto &camera : cameras_) {
      std::cout << PoseWithCovariance::to_str(camera.camera_f_world_) << std::endl;
    }
  }

  gtsam::Cal3_S2 CamerasModel::get_Cal3_S2()
  {
    return gtsam::Cal3_S2{calibration_.fx(), calibration_.fy(),
                          calibration_.skew(),
                          calibration_.px(), calibration_.py()};
  }

  static std::vector<std::vector<CornersFImageModel>> gen_corners_f_images(const MarkersModel &markers,
                                                                           const CamerasModel &cameras)
  {
    std::vector<std::vector<CornersFImageModel>> corners_f_images;

    for (auto &camera : cameras.cameras_) {
      std::vector<CornersFImageModel> per_camera;
      for (auto &marker : markers.markers_) {
        std::vector<gtsam::Point2> corners_f_image{};

        // Project the corners of this marker into this camera's image plane.
        for (auto &corner_f_world : marker.corners_f_world_) {
          try {
            auto corner_f_image = cameras.project_func_(camera.camera_f_world_,
                                                        corner_f_world,
                                                        boost::none);

            // If the point is outside of the image boundary, then don't save any of the points. This
            // simulates when a marker can't be seen by a camera.
            if (corner_f_image.x() < 0 || corner_f_image.x() >= 2 * cameras.calibration_.px() ||
                corner_f_image.y() < 0 || corner_f_image.y() >= 2 * cameras.calibration_.py()) {
              corners_f_image.clear();
              break;
            }

            corners_f_image.emplace_back(corner_f_image);
          }

            // If the point can't be projected, then don't save any of the points. This
            // simulates when a marker can't be seen by a camera.
          catch (gtsam::CheiralityException &e) {
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
    cfg_{markers_configuration, cameras_configuration, camera_type},
    markers_{cfg_},
    cameras_{cfg_},
    corners_f_images_{gen_corners_f_images(markers_, cameras_)}
  {}

  Model::Model(ModelConfig cfg) :
    cfg_{std::move(cfg)},
    markers_{cfg_},
    cameras_{cfg_},
    corners_f_images_{gen_corners_f_images(markers_, cameras_)}
  {}

  void Model::print_corners_f_image()
  {
    std::cout << "corners_f_images" << std::endl;
    for (auto &per_camera : corners_f_images_) {
      for (auto &per_marker : per_camera) {

        if (per_marker.marker_idx_ == 0) {
          auto &camera = cameras_.cameras_[per_marker.camera_idx_];
          std::cout << "camera_" << camera.camera_idx_
                    << PoseWithCovariance::to_str(camera.camera_f_world_) << std::endl;
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
    std::cout << std::endl;
  }
}
