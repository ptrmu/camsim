
#include "fvlam/transform3_with_covariance.hpp"

#include "gtsam/geometry/Rot3.h"

namespace fvlam
{

  template<>
  gtsam::Rot3 fromRotate3(const Rotate3 &other)
  {
    return gtsam::Rot3{other.q()};
  }
}
