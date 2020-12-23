
#ifndef PFM_RUN_HPP
#define PFM_RUN_HPP

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

namespace camsim
{
  class PfmModel;

  int pfm_run(int model_id);

  void pfm_resection_gtsam(const gtsam::Cal3_S2 &camera_calibration,
                           const std::vector<gtsam::Point2> &corners_f_image,
                           const std::vector<gtsam::Point3> &corners_f_world,
                           const gtsam::Pose3 &camera_f_world_initial,
                           const gtsam::SharedNoiseModel &measurement_noise,
                           gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance);

  void pfm_resection_opencv(const gtsam::Cal3_S2 &camera_calibration,
                            const std::vector<gtsam::Point2> &corners_f_image,
                            const std::vector<gtsam::Point3> &corners_f_world,
                            const gtsam::Pose3 &camera_f_world_initial,
                            const gtsam::SharedNoiseModel &measurement_noise,
                            gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance);

  void pfm_resection_projection(const gtsam::Cal3_S2 &camera_calibration,
                                const std::vector<gtsam::Point2> &corners_f_image,
                                const std::vector<gtsam::Point3> &corners_f_world,
                                const gtsam::Pose3 &camera_f_world_initial,
                                const gtsam::SharedNoiseModel &measurement_noise,
                                gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance);

  void pfm_resection_monte_carlo(const gtsam::Cal3_S2 &camera_calibration,
                                 const std::vector<gtsam::Point2> &corners_f_image,
                                 const std::vector<gtsam::Point3> &corners_f_world,
                                 const gtsam::Pose3 &camera_f_world_initial,
                                 const gtsam::SharedNoiseModel &measurement_noise,
                                 gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance);

  int pfm_simple_rotation_example();

  int pfm_optimize_pose3();
}

#endif //PFM_RUN_HPP
