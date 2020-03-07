
#ifndef _SFM_POSE_WITH_COVARIANCE_HPP
#define _SFM_POSE_WITH_COVARIANCE_HPP

#include <gtsam/3rdparty/Eigen/Eigen/StdVector>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"

namespace gtsam
{
  class NonlinearFactorGraph;

  class Values;

  class Marginals;
}

namespace camsim
{
  struct PoseWithCovariance
  {
    typedef std::vector<camsim::PoseWithCovariance, Eigen::aligned_allocator<PoseWithCovariance>> StdVector;

    const std::uint64_t key_;
    const int id_; // remove someday
    const gtsam::Pose3 pose_;
    const gtsam::Matrix6 cov_;

    PoseWithCovariance(std::uint64_t key, const gtsam::Pose3 &pose, const gtsam::Matrix6 &cov) :
      key_{key}, id_{static_cast<int>(key)}, pose_{pose}, cov_{cov}
    {}

    static PoseWithCovariance Extract(const gtsam::Values &result,
                                      const gtsam::Marginals *marginals,
                                      gtsam::Key key);

    std::string to_str() const; //
    static std::string to_str(const gtsam::Pose3 &pose); //
    static std::string to_str(const gtsam::Matrix6 &cov); //
    static std::string to_eigenvalues_str(const gtsam::Matrix6 &cov); //
    static std::string to_stddev_str(const gtsam::Matrix6 &cov);//
  };

  struct PointWithCovariance
  {
    typedef std::array<PointWithCovariance, 4> FourPoints;

    const std::uint64_t key_;
    const gtsam::Point3 point_;
    const gtsam::Matrix3 cov_;

    PointWithCovariance(std::uint64_t key, const gtsam::Point3 &point, const gtsam::Matrix3 &cov) :
      key_{key}, point_{point}, cov_{cov}
    {}

    static PointWithCovariance Extract(const gtsam::Values &result,
                                       const gtsam::Marginals *marginals,
                                       gtsam::Key key);

    static FourPoints Extract4(const gtsam::Values &result,
                               const gtsam::Marginals *marginals,
                               gtsam::Key marker_key);

    std::string to_str() const; //
    static std::string to_str(const gtsam::Point3 &point); //
    static std::string to_str(const gtsam::Matrix3 &cov); //
    static std::string to_eigenvalues_str(const gtsam::Matrix3 &cov); //
    static std::string to_stddev_str(const gtsam::Matrix3 &cov); //
  };
}

#endif //_SFM_POSE_WITH_COVARIANCE_HPP
