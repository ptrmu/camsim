
#include "sfm_run.hpp"

namespace camsim
{
  int sfm_run()
  {
    return sfm_gtsam_example();
  }
}

int main()
{
  return camsim::sfm_run();
}
