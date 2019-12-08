
#include "pfp_run.hpp"

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"

using namespace gtsam;

namespace camsim
{
  void pfp_pose_unit_test()
  {
    Pose3 base_pose3{};

    auto adjoint = base_pose3.AdjointMap();
    
  }
}
