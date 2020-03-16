
#include <functional>

#include "cal_run.hpp"
#include "cal_solver_runner.hpp"
#include "calibration_model.hpp"

namespace camsim
{
  using PoseGeneratorFunc = std::function<std::vector<gtsam::Pose3>(void)>;

  PoseGeneratorFunc gen_poses_func(std::vector<gtsam::Pose3> poses)
  {
    return [poses]()
    {
      return poses;
    };
  }

  PoseGeneratorFunc gen_poses_func_origin_looking_up()
  {
    return gen_poses_func({gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., 0.),
                                        gtsam::Point3{0., 0., 0.}}});
  }

  PoseGeneratorFunc gen_poses_func_heiko_calibration_poses()
  {
    return gen_poses_func({gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                        gtsam::Point3{0., 0., 0.18}},
                           gtsam::Pose3{gtsam::Rot3::RzRyRx(0.75 * M_PI, 0., M_PI),
                                        gtsam::Point3{0., -0.18, 0.18}},
                           gtsam::Pose3{gtsam::Rot3::RzRyRx(-0.75 * M_PI, 0., M_PI),
                                        gtsam::Point3{0., 0.18, 0.18}},
                           gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0.25 * M_PI, M_PI),
                                        gtsam::Point3{-0.195, 0., 0.195}},
                           gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, -0.25 * M_PI, M_PI),
                                        gtsam::Point3{0.195, 0., 0.195}}});
  }


  int cal_solver_opencv_checkerboard()
  {
    ModelConfig model_config{gen_poses_func_origin_looking_up(),
                             gen_poses_func_heiko_calibration_poses(),
                             camsim::CameraTypes::simulation,
                             0.1775};

    CheckerboardConfig ch_cfg(12, 9, 0.030);

    CheckerboardCalibrationModel ccm(model_config, ch_cfg);

    ccm.print_junctions_f_image();
  }
}
