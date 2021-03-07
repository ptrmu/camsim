

#include "fvlam/model.hpp"
#include "opencv2/core.hpp"

namespace fvlam
{
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


  std::vector<Transform3> CamerasGen::SpinAboutZAtOriginFacingOut(int n)
  {
    static Transform3 base{fvlam::Rotate3::RzRyRx(M_PI_2, 0., M_PI_2), fvlam::Translate3{0, 0, 0}};
    return rotate_around_z(n, base);
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
      auto cv_camera_calibration = camera_info.to<fvlam::CvCameraCalibration>();

      auto cv_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        cv_camera_calibration,
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