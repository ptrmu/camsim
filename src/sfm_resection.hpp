
#ifndef _SFM_RESECTION_HPP
#define _SFM_RESECTION_HPP

#include <tuple>
#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  std::tuple<gtsam::Pose3, gtsam::Matrix6> sfm_resection();
}
#endif //_SFM_RESECTION_HPP
