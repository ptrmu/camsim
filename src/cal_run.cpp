
#include <cstdlib>

#include "cal_run.hpp"

#include "calibration_model.hpp"
#include "model.hpp"

namespace camsim
{
  int cal_run()
  {
    int n_cameras = 64;
    int n_markers = 8;
    
    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{n_markers, 2.},
                             PoseGens::SpinAboutZAtOriginFacingOut{n_cameras},
                             camsim::CameraTypes::simulation,
                             0.1775};

//    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingAlongZ{n_markers, 2., 2., false},
//                            PoseGens::CircleInXYPlaneFacingAlongZ{n_cameras, 2., 0., true},
//                            camsim::CameraTypes::simulation,
//                            0.1775};

    Model model{model_config};


    CheckerboardConfig ch_cfg{12, 9, 0.030};

    CheckerboardCalibrationModel ccm(model_config, ch_cfg);

    CharucoboardConfig ar_cfg{12, 9, 0.030, false, 0.0225};

    CharucoboardCalibrationModel acm(model_config, ar_cfg);

    return EXIT_SUCCESS;
  }
}