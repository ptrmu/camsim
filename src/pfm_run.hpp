
#ifndef PFM_RUN_HPP
#define PFM_RUN_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace camsim
{
  class PfmModel;

  void pfm_gtsam_resection(const PfmModel &pfm_model, const gtsam::SharedNoiseModel &measurement_noise,
                           gtsam::Pose3 &camera_f_world, gtsam::Matrix &camera_f_world_covariance);

  void pfm_opencv_resection(const PfmModel &pfm_model, const gtsam::SharedNoiseModel &measurement_noise,
                            gtsam::Pose3 &camera_f_world, gtsam::Matrix &camera_f_world_covariance);
}

#endif //PFM_RUN_HPP
