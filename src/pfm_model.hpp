#ifndef PFM_MODEL_HPP
#define PFM_MODEL_HPP

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace camsim
{
  struct PfmModel
  {
    const gtsam::Cal3_S2 camera_calibration_;
    const gtsam::Pose3 camera_f_world_;
    const gtsam::Pose3 marker_f_world_;
    const double marker_size_;
    const gtsam::Pose3 camera_f_marker_;
    const gtsam::SimpleCamera camera_;
    const std::vector<gtsam::Point3> corners_f_world_;
    std::vector<gtsam::Point2> corners_f_image_;

    PfmModel(const gtsam::Cal3_S2 &camera_calibration,
             const gtsam::Pose3 &camera_f_world,
             const gtsam::Pose3 &marker_f_world,
             const double &marker_size);
  };

  int pfm_model_test();
}
#endif //PFM_MODEL_HPP
