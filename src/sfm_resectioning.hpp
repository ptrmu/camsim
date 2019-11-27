
#ifndef _SFM_RESECTION_HPP
#define _SFM_RESECTION_HPP

#include <tuple>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include "pose_with_covariance.hpp"

namespace gtsam
{
  class Cal3DS2;
}

namespace camsim
{
  class CalcCameraPoseImpl;

  class CalcCameraPose
  {
    std::unique_ptr<CalcCameraPoseImpl> impl_;

  public:
    CalcCameraPose(const gtsam::Cal3DS2 &K,
                   const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                     const gtsam::Point3 &,
                                                     boost::optional<gtsam::Matrix &>)> &project_func,
                   const gtsam::SharedNoiseModel &measurement_noise,
                   const std::vector<gtsam::Point3> &corners_f_marker);

    ~CalcCameraPose();

    PoseWithCovariance camera_f_marker(
      int marker_id,
      const std::vector<gtsam::Point2> &corners_f_image);
  };
}

int sfm_run_resectioning();

#endif //_SFM_RESECTION_HPP
