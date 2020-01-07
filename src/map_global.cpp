

#include "map_run.hpp"
#include "model.hpp"

namespace camsim
{
  void map_global()
  {
    camsim::Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingOrigin{8, 2.},
                                    PoseGens::SpinAboutZAtOriginFacingOut{32},
                                    camsim::CameraTypes::distorted_camera,
                                    1}};
    model.print_corners_f_image();
  }
}