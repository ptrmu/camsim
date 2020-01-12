
#include "pose_with_covariance.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>

namespace camsim
{

  class NumFmt
  {
    int width_;
    int precision_;
    double val_{};
    bool set_{false};

  public:
    NumFmt(int width, int precision)
      : width_(width), precision_(precision)
    {
    }

    NumFmt &operator()(double val)
    {
      set_ = true;
      val_ = val;
    }

    friend std::ostream &
    operator<<(std::ostream &dest, NumFmt &fmt)
    {
//      dest.setf(std::ios_base::fixed, std::ios_base::floatfield);
      dest.unsetf(std::ios_base::floatfield);
      dest.precision(fmt.precision_);
      dest.width(fmt.width_);
      if (fmt.set_) {
        fmt.set_ = false;
        dest << (abs(fmt.val_) < 1.e-8 ? 0.0 : fmt.val_);
      }
      return dest;
    }
  };

  PoseWithCovariance PoseWithCovariance::Extract(gtsam::NonlinearFactorGraph &graph,
    gtsam::Values &result,
    gtsam::Key key)
  {
    gtsam::Marginals marginals(graph, result);
    return PoseWithCovariance{static_cast<int>(gtsam::symbolIndex(key)),
                              result.at<gtsam::Pose3>(key),
                              marginals.marginalCovariance(key)};
  }

  std::string PoseWithCovariance::to_str() const
  {
    return to_str(pose_).append("\n").append(to_str(cov_));
  }

  std::string PoseWithCovariance::to_str(const gtsam::Pose3 &pose)
  {
    NumFmt nf(9, 3);
    auto r = pose.rotation().xyz();
    auto &t = pose.translation();
    std::stringstream ss{};
    ss << nf(r(0)) << " " << nf(r(1)) << " " << nf(r(2)) << " "
       << nf(t(0)) << " " << nf(t(1)) << " " << nf(t(2));
    return ss.str();
  }

  std::string PoseWithCovariance::to_str(const gtsam::Matrix6 &cov)
  {
    NumFmt nf(9, 3);
    auto &v = cov;
    std::stringstream ss{};
    ss << nf(v(0, 0)) << std::endl
       << nf(v(1, 0)) << " " << nf(v(1, 1)) << std::endl
       << nf(v(2, 0)) << " " << nf(v(2, 1)) << " " << nf(v(2, 2)) << std::endl
       << nf(v(3, 0)) << " " << nf(v(3, 1)) << " " << nf(v(3, 2)) << " " << nf(v(3, 3)) << std::endl
       << nf(v(4, 0)) << " " << nf(v(4, 1)) << " " << nf(v(4, 2)) << " " << nf(v(4, 3)) << " " << nf(v(4, 4))
       << std::endl
       << nf(v(5, 0)) << " " << nf(v(5, 1)) << " " << nf(v(5, 2)) << " " << nf(v(5, 3)) << " " << nf(v(5, 4)) << " "
       << nf(v(5, 5));
    return ss.str();
  }

  static std::string to_eigen_values_str(const gtsam::Matrix6 &cov)
  {
    return std::string{};
  }

}
