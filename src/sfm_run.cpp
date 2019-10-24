
#include "sfm_run.hpp"

#include "sfm_model.hpp"

namespace camsim
{
  int sfm_run()
  {
    SfmModel sfm_model{MarkersConfigurations::square_around_origin_xy_plane,
                       CamerasConfigurations::square_around_z_axis};

    return EXIT_SUCCESS;
  }
}

int main()
{
  return camsim::sfm_gtsam_slam_example();
//  return camsim::sfm_gtsam_example();
//  return camsim::sfm_run();
}
