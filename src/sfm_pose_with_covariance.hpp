
#ifndef _SFM_POSE_WITH_COVARIANCE_HPP
#define _SFM_POSE_WITH_COVARIANCE_HPP

#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  struct SfmPoseWithCovariance
  {
    const int id_;
    const gtsam::Pose3 pose_;
    const gtsam::Matrix6 cov_;

    SfmPoseWithCovariance(int id, const gtsam::Pose3 &pose, const gtsam::Matrix6 &cov) :
      id_{id}, pose_{pose}, cov_{cov}
    {}
  };
}

#endif //_SFM_POSE_WITH_COVARIANCE_HPP
