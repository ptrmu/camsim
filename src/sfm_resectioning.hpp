
#ifndef _SFM_RESECTION_HPP
#define _SFM_RESECTION_HPP

#include <tuple>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include "sfm_pose_with_covariance.hpp"

namespace gtsam
{
  class Cal3_S2;
}

namespace camsim
{
  class CalcCameraPoseImpl;

  class CalcCameraPose
  {
    std::unique_ptr<CalcCameraPoseImpl> impl_;

  public:
    CalcCameraPose(const gtsam::Cal3_S2 &K,
                   const gtsam::SharedNoiseModel &measurement_noise,
                   const std::vector<gtsam::Point3> &corners_f_marker);

    ~CalcCameraPose();

    SfmPoseWithCovariance camera_f_marker(
      int marker_id,
      const std::vector<gtsam::Point2> &corners_f_image);
  };
}

int sfm_run_resectioning();

#endif //_SFM_RESECTION_HPP