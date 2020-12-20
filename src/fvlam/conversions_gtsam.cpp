
#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

namespace fvlam
{
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

  template<>
  Marker::CornersMatrix Marker::to_corners_f_world<Marker::CornersMatrix>(double marker_length) const
  {
    return calc_corners_f_world(marker_length);
  }

  template<>
  CameraInfo CameraInfo::from<gtsam::Cal3DS2>(const gtsam::Cal3DS2 &other)
  {
    return CameraInfo{other.fx(), other.fy(),
                      other.skew(),
                      other.px(), other.py()}; // todo: add distortion coefficients
  }
}
