
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>

namespace fvlam
{

// ==============================================================================
// from fvlam/transform3_with_covariance.hpp
// ==============================================================================

  template<>
  Translate2 Translate2::from<gtsam::Vector2>(const gtsam::Vector2 &other)
  {
    return Translate2{other};
  }

  template<>
  gtsam::Vector2 Translate2::to<gtsam::Vector2>() const
  {
    return t();
  }

  template<>
  Translate3 Translate3::from<gtsam::Vector3>(const gtsam::Vector3 &other)
  {
    return Translate3{other};
  }

  template<>
  gtsam::Vector3 Translate3::to<gtsam::Vector3>() const
  {
    return t_;
  }

  template<>
  Rotate3 Rotate3::from<gtsam::Rot3>(const gtsam::Rot3 &other)
  {
    return Rotate3{other.matrix()};
  }

  template<>
  gtsam::Rot3 Rotate3::to<gtsam::Rot3>() const
  {
    return gtsam::Rot3(rotation_matrix());
  }

  template<>
  Transform3 Transform3::from<gtsam::Pose3>(const gtsam::Pose3 &other)
  {
    return Transform3{Rotate3::from(other.rotation()), Translate3::from(other.translation())};
  }

  template<>
  gtsam::Pose3 Transform3::to<gtsam::Pose3>() const
  {
    return gtsam::Pose3{r_.to<gtsam::Rot3>(), t_.to<gtsam::Vector3>()};
  }

// ==============================================================================
// from fvlam/camera_info.hpp
// ==============================================================================

 template<>
  CameraInfo CameraInfo::from<gtsam::Cal3DS2>(const gtsam::Cal3DS2 &other)
  {
    return CameraInfo{other.fx(), other.fy(),
                      other.skew(),
                      other.px(), other.py(),
                      other.k1(), other.k2(),
                      other.p1(), other.p2(),
                      0.0};
  }

  template<>
  gtsam::Cal3DS2 CameraInfo::to<gtsam::Cal3DS2>() const
  {
    return gtsam::Cal3DS2{
      camera_matrix_(0, 0),  // fx
      camera_matrix_(1, 1),  // fy
      camera_matrix_(0, 1), // s
      camera_matrix_(0, 2),  // u0
      camera_matrix_(1, 2),  // v0
      dist_coeffs_(0), // k1
      dist_coeffs_(1), // k2
      dist_coeffs_(2), // p1
      dist_coeffs_(3) // p2
    };
  }

// ==============================================================================
// from fvlam/marker_map.hpp
// ==============================================================================

  template<>
  std::vector<gtsam::Vector3> Marker::to_corners_f_world<std::vector<gtsam::Vector3>>(double marker_length) const
  {
    auto corners_f_world = calc_corners3_f_world(marker_length);
    return std::vector<gtsam::Vector3>{
      gtsam::Vector3{corners_f_world[0].t()},
      gtsam::Vector3{corners_f_world[1].t()},
      gtsam::Vector3{corners_f_world[2].t()},
      gtsam::Vector3{corners_f_world[3].t()}
    };
  }

// ==============================================================================
// from fvlam/observation.hpp
// ==============================================================================


// ==============================================================================
// fvlam::Marker conversions that require Observation conversions so
// have to be after those conversions in the file.
// ==============================================================================

  template<>
  Marker::ProjectFunction Marker::project_t_world_marker<gtsam::Cal3DS2>(
    const gtsam::Cal3DS2 &camera_calibration,
    const Transform3 &t_world_camera,
    double marker_length)
  {
    // can optimize this later

    return [
      camera_calibration,
      t_world_camera,
      marker_length]
      (const Marker &marker) -> Observation
    {
      auto gtsam_t_world_camera = t_world_camera.to<gtsam::Pose3>();
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{gtsam_t_world_camera,
                                                         camera_calibration};

      Observation::Array corners_f_image;
      auto corners_f_world = marker.to_corners_f_world<std::vector<gtsam::Point3>>(marker_length);
      for (int i = 0; i < Observation::ArraySize; i += 1) {
        auto &corner_f_world = corners_f_world[i];

        try {
          auto corner_f_image = camera.project(corner_f_world);

          // If the point is outside of the image boundary, then don't save any of the points. This
          // simulates when a marker can't be seen by a camera.
          if (corner_f_image.x() < 0 || corner_f_image.x() >= 2 * camera_calibration.px() ||
              corner_f_image.y() < 0 || corner_f_image.y() >= 2 * camera_calibration.py()) {
            return Observation{marker.id()};
          }


          corners_f_image[i]= Translate2::from(corner_f_image);
        }

          // If the point can't be projected, then don't save any of the points. This
          // simulates when a marker can't be seen by a camera.
        catch (gtsam::CheiralityException &e) {
          return Observation{marker.id()};
        }
      }

      return Observation{marker.id(), corners_f_image};
    };
  }
//
//  template<>
//  Marker::SolveFunction Marker::solve_t_camera_marker<gtsam::Cal3DS2>(
//    const gtsam::Cal3DS2 &camera_calibration, double marker_length)
//  {
//    return [
//      camera_matrix = camera_calibration.first,
//      dist_coeffs = camera_calibration.second,
//      marker_length]
//      (const Observation &observation) -> Marker
//    {
//      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame.
//      auto corners_f_marker{Marker::to_corners_f_marker<std::vector<cv::Point3d>>(marker_length)};
//      auto corners_f_image{observation.to<std::vector<cv::Point2d>>()};
//
//      // Figure out marker pose.
//      cv::Vec3d rvec, tvec;
//      cv::solvePnP(corners_f_marker, corners_f_image,
//                   camera_matrix, dist_coeffs,
//                   rvec, tvec);
//
//      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
//      // camera coordinate system". In this case the marker frame is the model coordinate system.
//      // So rvec, tvec are the transformation t_camera_marker.
//      return Marker{bservation.id(),
//                    Transform3WithCovariance(Transform3(Rotate3::from(rvec), Translate3::from(tvec)))};
//    };
//  }

}
