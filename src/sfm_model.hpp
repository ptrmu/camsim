
#ifndef _SFM_MODEL_HPP
#define _SFM_MODEL_HPP

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>

namespace camsim
{
  struct SfmModel
  {
    const gtsam::Cal3_S2 camera_calibration_;
    const std::vector<gtsam::Pose3> camera_f_worlds_;
    const std::vector<gtsam::Pose3> marker_f_worlds_;
    const double marker_size_;
    const std::vector<gtsam::SimpleCamera> cameras_;
    const std::vector<std::vector<gtsam::Point3>> corners_f_worlds_;
    const std::vector<std::vector<std::vector<gtsam::Point2>>> corners_f_images_;

    SfmModel(const gtsam::Cal3_S2 &camera_calibration,
             const std::vector<gtsam::Pose3> &camera_f_worlds,
             const std::vector<gtsam::Pose3> &marker_f_worlds,
             const double &marker_size);
  };
}
#endif //_SFM_MODEL_HPP
