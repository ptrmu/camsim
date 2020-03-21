
#include <cstdlib>

#include "cal_run.hpp"

#include "calibration_model.hpp"
#include "model.hpp"
#include "cal_solver_runner.hpp"

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

  int cal_solver()
  {
    ModelConfig model_config{PoseGens::gen_poses_func_origin_looking_up(),
                             PoseGens::gen_poses_func_heiko_calibration_poses(),
                             camsim::CameraTypes::simulation,
                             0.1775};

    CheckerboardConfig ch_cfg(12, 9, 0.030);
    CheckerboardCalibrationModel ccm(model_config, ch_cfg);

    ccm.print_junctions_f_image();

    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 0.001;
    double u_noise_sigma = 1.0;

    CheckerboardSolverRunner solver_runner{ccm,
                                           (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                             gtsam::Vector3::Constant(t_sigma)).finished(),
                                           (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                             gtsam::Vector3::Constant(t_sigma)).finished(),
                                           gtsam::Vector2::Constant(u_sampler_sigma),
                                           gtsam::Vector2::Constant(u_noise_sigma),
                                           false};


    solver_runner(solver_opencv_factory<CheckerboardCalibrationModel>());
  }
}

