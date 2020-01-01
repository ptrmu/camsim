

#include "map_run.hpp"
#include "model.hpp"

namespace camsim
{
  void map_global()
  {
    camsim::Model model{camsim::MarkersConfigurations::upright_circle_around_z_axis,
                        camsim::CamerasConfigurations::center_looking_x,
                        camsim::CameraTypes::distorted_camera};

  }
}