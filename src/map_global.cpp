

#include "map_run.hpp"
#include "model.hpp"

namespace camsim
{
  void map_global()
  {
    camsim::Model model{camsim::MarkersConfigurations::upright_circle_around_z_axis,
                        camsim::CamerasConfigurations::center_looking_x,
                        camsim::CameraTypes::distorted_camera};
    model.print_corners_f_image();

    camsim::Model model_1{model, gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., M_PI_2),
                                              gtsam::Point3(0., 0., 0.)}};
    model_1.print_corners_f_image();
  }
}