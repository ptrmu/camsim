
#ifndef PFM_RUN_HPP
#define PFM_RUN_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace camsim
{
  class PfmModel;

  void pfm_gtsam_resection(const gtsam::Cal3_S2 &camera_calibration,
                           const std::vector<gtsam::Point2> &corners_f_image,
                           const std::vector<gtsam::Point3> &corners_f_world,
                           const gtsam::Pose3 &camera_f_world_initial,
                           const gtsam::SharedNoiseModel &measurement_noise,
                           gtsam::Pose3 &camera_f_world, gtsam::Matrix &camera_f_world_covariance);

  void pfm_opencv_resection(const PfmModel &pfm_model, const gtsam::SharedNoiseModel &measurement_noise,
                            gtsam::Pose3 &camera_f_world, gtsam::Matrix &camera_f_world_covariance);

  int pfm_simple_rotation_example();

  int pfm_optimize_pose3();
}

#endif //PFM_RUN_HPP
