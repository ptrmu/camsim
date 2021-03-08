

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include "opencv2/core.hpp"

namespace fvlam
{
  MapEnvironment MapEnvironmentGen::Default()
  {
    return fvlam::MapEnvironment{"TestMap", 0, 0.2};
  }

  CameraInfoMap CameraInfoMapGen::DualCamera()
  {
    auto camera_info_base = CameraInfo{475, 475, 0, 400, 300};
    auto camera_info0 = CameraInfo("left", camera_info_base, Transform3{Rotate3{}, Translate3{-0.2, 0, 0}});
    auto camera_info1 = CameraInfo("right", camera_info_base, Transform3{Rotate3{}, Translate3{0.2, 0, 0}});

    auto camera_info_map = CameraInfoMap{};
    camera_info_map.emplace(camera_info0.imager_frame_id(), camera_info0);
    camera_info_map.emplace(camera_info1.imager_frame_id(), camera_info1);

    return camera_info_map;
  }

  static std::vector<fvlam::Transform3> rotate_around_z(int n, const fvlam::Transform3 &base)
  {
    std::vector<fvlam::Transform3> pose_f_worlds{};
    double delta_rotz = M_PI * 2 / n;
    for (int i = 0; i < n; i += 1) {
      auto rotz = delta_rotz * i;
      pose_f_worlds.emplace_back(
        fvlam::Transform3(fvlam::Rotate3::RzRyRx(0., 0., rotz), fvlam::Translate3{0, 0, 0}) * base);
    }
    return pose_f_worlds;
  }

  static std::vector<Marker> markers_from_transform3s(std::vector<Transform3> transform3s,
                                                      std::uint64_t id_base)
  {
    std::vector<Marker> markers;
    for (auto &transform3 : transform3s) {
      markers.emplace_back(Marker{id_base++, Transform3WithCovariance{transform3}});
    }
    return markers;
  }

  std::vector<Transform3> CamerasGen::SpinAboutZAtOriginFacingOut(int n)
  {
    Transform3 base{fvlam::Rotate3::RzRyRx(M_PI_2, 0., M_PI_2), fvlam::Translate3{0, 0, 0}};
    return rotate_around_z(n, base);
  }

  std::vector<Transform3> CamerasGen::LookingDownZ(double z)
  {
    auto p = Transform3{fvlam::Rotate3::RzRyRx(M_PI, 0., 0.), fvlam::Translate3{0, 0, z}};
    return std::vector<Transform3>{p};
  }

  template<>
  std::vector<Marker> MarkersGen::CircleInXYPlaneFacingOrigin(int n, double radius)
  {
    return markers_from_transform3s(rotate_around_z(
      n, Transform3{Rotate3::RzRyRx(M_PI_2, 0., -M_PI_2), Translate3{radius, 0., 0.}}), 0);
  }

  template<>
  std::vector<Marker> MarkersGen::OriginLookingUp()
  {
    return markers_from_transform3s(std::vector<Transform3>{Transform3{}}, 0);
  }

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  ObservationsSynced MarkerObservations::gen_observations_synced(const MapEnvironment &map_environment,
                                                                 const CameraInfoMap &camera_info_map,
                                                                 const Transform3 &t_map_camera,
                                                                 const std::vector<Marker> &markers)
  {
    auto observations_synced = ObservationsSynced{Stamp{}, "camera_frame"};

    for (auto &camera_info_pair : camera_info_map) {
      const CameraInfo &camera_info = camera_info_pair.second;
      auto gtsam_camera_calibration = camera_info.to<gtsam::Cal3DS2>();

      auto cv_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        t_map_camera * camera_info.t_camera_imager(),
        map_environment.marker_length());

      auto observations = Observations{camera_info.imager_frame_id()};
      for (auto &marker : markers) {
        observations.emplace_back(cv_project_t_world_marker_function(marker));
      }
      observations_synced.emplace_back(observations);
    }

    return observations_synced;
  }

}