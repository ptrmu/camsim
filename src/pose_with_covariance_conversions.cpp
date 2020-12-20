
#include "fvlam/transform3_with_covariance.hpp"

#include "gtsam/geometry/Rot3.h"
#include "gtsam/geometry/Pose3.h"

namespace fvlam
{
#if 0
  template<>
  gtsam::Rot3 Rotate3::to<gtsam::Rot3>() const
  {
    return gtsam::Rot3{q()};
  }

  template<>
  Transform3 Transform3::from<gtsam::Pose3>(const gtsam::Pose3 &other)
  {
    return Transform3(Rotate3(Eigen::Quaterniond(other.rotation().matrix())), Translate3(other.translation()));
  }

  template<>
  gtsam::Pose3 Transform3::to<gtsam::Pose3>() const
  {
    return gtsam::Pose3{gtsam::Rot3(r().q()), gtsam::Point3(t().t())};
  }
#endif
}
