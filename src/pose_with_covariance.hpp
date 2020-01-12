
#ifndef _SFM_POSE_WITH_COVARIANCE_HPP
#define _SFM_POSE_WITH_COVARIANCE_HPP

#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"

namespace gtsam
{
  class NonlinearFactorGraph;

  class Values;
}

namespace camsim
{
  struct PoseWithCovariance
  {
    const int id_;
    const gtsam::Pose3 pose_;
    const gtsam::Matrix6 cov_;

    PoseWithCovariance(int id, const gtsam::Pose3 &pose, const gtsam::Matrix6 &cov) :
      id_{id}, pose_{pose}, cov_{cov}
    {}

    static PoseWithCovariance Extract(gtsam::NonlinearFactorGraph &graph, gtsam::Values &result, gtsam::Key key);

    std::string to_str() const;

    static std::string to_str(const gtsam::Pose3 &pose);

    static std::string to_str(const gtsam::Matrix6 &cov);
  };

  struct PointWithCovariance
  {
    const int id_;
    const int corner_id_;
    const gtsam::Point3 point_;
    const gtsam::Matrix3 cov_;

    PointWithCovariance(int id, int corner_id, const gtsam::Point3 &point, const gtsam::Matrix3 &cov) :
      id_{id}, corner_id_{corner_id}, point_{point}, cov_{cov}
    {}

    std::string to_str() const;

    static std::string to_str(const gtsam::Point3);

    static std::string to_str(const gtsam::Matrix3);
  };
}

#endif //_SFM_POSE_WITH_COVARIANCE_HPP
