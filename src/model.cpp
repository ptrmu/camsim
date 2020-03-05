
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

  std::vector<gtsam::Pose3> PoseGens::CircleInXYPlaneFacingAlongZ::operator()() const
  {
    auto poses_in_circle = rotate_around_z(n_, gtsam::Pose3{gtsam::Rot3{},
                                                            gtsam::Point3{radius_, 0., 0.}});
    auto rot = facing_z_plus_not_z_negative_ ? gtsam::Rot3{} : gtsam::Rot3::RzRyRx(M_PI, 0., M_PI_2);
    for (auto &pose : poses_in_circle) {
      pose = gtsam::Pose3{rot, gtsam::Point3{pose.translation().x(), pose.translation().y(), z_offset_}};
    }
    return poses_in_circle;
  }

  std::vector<gtsam::Pose3> PoseGens::CubeAlongZFacingOrigin::operator()() const
  {
    std::vector<gtsam::Pose3> pose_f_worlds{};

    auto side_offset = -side_length_ / 2.;
    auto side_delta = side_length_ / (per_side_ - 1);
    for (int ix = 0; ix < per_side_; ix += 1) {
      for (int iy = 0; iy < per_side_; iy += 1) {
        for (int iz = 0; iz < per_side_; iz += 1) {
          pose_f_worlds.emplace_back(gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                                  gtsam::Point3{ix * side_delta + side_offset,
                                                                iy * side_delta + side_offset,
                                                                iz * side_delta + side_offset + z_offset_}});
        }
      }
    }

    return pose_f_worlds;
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

  std::size_t CornerModel::index() const
  {
    return gtsam::Symbol(key_).index();
  }


  std::uint64_t CornerModel::default_key(int corner_idx)
  {
    return corner_key(MarkerModel::default_key(), corner_idx);
  }

  std::uint64_t CornerModel::corner_key(std::uint64_t marker_key, int corner_idx)
  {
    auto marker_index = gtsam::Symbol{marker_key}.index();
    static char codes[] = {'i', 'j', 'k', 'l'};
    return gtsam::Symbol(codes[corner_idx % sizeof(codes)], marker_index);
  }

  static std::array<CornerModel, 4> gen_corners(std::uint64_t marker_key,
                                                const gtsam::Pose3 &marker_f_world,
                                                const std::vector<gtsam::Point3> &corners_f_marker)
  {
    auto marker_index{gtsam::Symbol{marker_key}.index()};
    return std::array<CornerModel, 4>{
      CornerModel(CornerModel::corner_key(marker_key, 0), marker_f_world * corners_f_marker[0]),
      CornerModel(CornerModel::corner_key(marker_key, 1), marker_f_world * corners_f_marker[1]),
      CornerModel(CornerModel::corner_key(marker_key, 2), marker_f_world * corners_f_marker[2]),
      CornerModel(CornerModel::corner_key(marker_key, 3), marker_f_world * corners_f_marker[3]),
    };
  }

  CornersModel::CornersModel(std::uint64_t marker_key_,
                             const gtsam::Pose3 &marker_f_world,
                             const std::vector<gtsam::Point3> &corners_f_marker) :
    corners_{gen_corners(marker_key_, marker_f_world, corners_f_marker)}
  {}

  std::size_t MarkerModel::index() const
  {
    return gtsam::Symbol(key_).index();
  }

  std::uint64_t MarkerModel::default_key()
  {
    return marker_key(0);
  }

  std::uint64_t MarkerModel::marker_key(std::size_t idx)
  {
    return gtsam::Symbol{'m', idx}.key();
  }


  std::uint64_t MarkerModel::marker_key_from_corner_key(std::uint64_t corner_key)
  {
    return marker_key(gtsam::Symbol{corner_key}.index());
  }

  static void add_sphere_points(const std::vector<gtsam::Point3> &vectors,
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

      auto marker_key{MarkerModel::marker_key(idx)};
      markers.emplace_back(MarkerModel{MarkerModel::marker_key(idx), marker_f_world,
                                       CornersModel(idx, marker_f_world, corners_f_marker),
                                       std::move(corners_f_world)});
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
      auto &corners = marker.corners_;
      std::cout << PoseWithCovariance::to_str(marker.marker_f_world_) << " ";
      std::cout << corners.corners_[0].point_f_world_
                << corners.corners_[1].point_f_world_
                << corners.corners_[2].point_f_world_
                << corners.corners_[3].point_f_world_ << std::endl;
    }
  }

  std::size_t CameraModel::index() const
  {
    return gtsam::Symbol(key_).index();
  }

  std::uint64_t CameraModel::default_key()
  {
    return camera_key(0);
  }

  std::uint64_t CameraModel::camera_key(std::size_t idx)
  {
    return gtsam::Symbol{'c', idx}.key();
  }

  static gtsam::Cal3DS2 gen_camera_calibration(const ModelConfig &cfg)
  {
    switch (cfg.camera_type_) {
      case CameraTypes::simulation:
        return gtsam::Cal3DS2{475, 475, 0, 400, 300, 0., 0.};
      default:
        return gtsam::Cal3DS2{1, 1, 0, 50, 50, 0., 0.};
    }
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
      cameras.emplace_back(CameraModel{CameraModel::camera_key(idx), camera_f_worlds[idx]});
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

    return [calibration](const gtsam::Pose3 &camera_f_xxx,
                         const gtsam::Point3 &point_f_xxx,
                         boost::optional<gtsam::Matrix &> H) -> gtsam::Point2
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_xxx, calibration};
      return camera.project(point_f_xxx, H);
    };
  }

  CamerasModel::CamerasModel(const ModelConfig &cfg) :
    cfg_{cfg},
    calibration_{gen_camera_calibration(cfg_)},
    project_func_{gen_project_func(cfg_, calibration_)},
    cameras_{gen_cameras(cfg, calibration_)}
  {
//    std::cout << "Cameras" << std::endl;
//    for (auto &camera : cameras_) {
//      std::cout << PoseWithCovariance::to_str(camera.camera_f_world_) << std::endl;
//    }
  }

  gtsam::Cal3_S2 CamerasModel::get_Cal3_S2() const
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
        for (auto &corner : marker.corners_.corners_) {
          try {
            auto corner_f_image = cameras.project_func_(camera.camera_f_world_,
                                                        corner.point_f_world_,
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
        per_camera.emplace_back(CornersFImageModel{marker.key_, camera.key_, corners_f_image});
      }

      corners_f_images.emplace_back(per_camera);
    }

    return corners_f_images;
  }

  std::size_t CornersFImageModel::marker_index() const
  {
    return gtsam::Symbol(marker_key_).index();
  }

  std::size_t CornersFImageModel::camera_index() const
  {
    return gtsam::Symbol(camera_key_).index();
  }

  BaseModel::BaseModel(const ModelConfig &cfg) :
    cfg_{cfg}
  {}

  Model::Model(const ModelConfig &cfg) :
    BaseModel{cfg},
    markers_{cfg_},
    cameras_{cfg_},
    corners_f_images_{gen_corners_f_images(markers_, cameras_)}
  {}

  void Model::print_corners_f_image()
  {
    std::cout << "corners_f_images" << std::endl;
    for (auto &per_camera : corners_f_images_) {
      for (auto &per_marker : per_camera) {

        if (per_marker.marker_index() == 0) {
          auto &camera = cameras_.cameras_[per_marker.camera_index()];
          std::cout << "camera_" << camera.index()
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
