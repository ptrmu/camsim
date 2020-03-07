
#include "pose_with_covariance.hpp"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/3rdparty/Eigen/Eigen/Eigenvalues>
#include "model.hpp"

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

  // Return a vector as a string containing a row of numbers
  static std::string to_row_str(gtsam::Vector v)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (auto p = v.data(); p < v.data() + v.size(); ++p) {
      if (p != v.data()) {
        ss << " ";
      }
      ss << nf(*p);
    }
    return ss.str();
  }

  // Return a matrix as a string containing the lower half of the matrix
  template<class FixedSizeMatrix>
  static std::string to_lower_str(const FixedSizeMatrix &m)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (int r = 0; r < m.rows(); r += 1) {
      if (r != 0) {
        ss << std::endl;
      }
      for (int c = 0; c <= r; c += 1) {
        if (c != 0) {
          ss << " ";
        }
        ss << nf(m(r, c));
      }
    }
    return ss.str();
  }

  PoseWithCovariance PoseWithCovariance::Extract(const gtsam::Values &result,
                                                 const gtsam::Marginals *marginals,
                                                 gtsam::Key key)
  {
    return PoseWithCovariance{key, result.at<gtsam::Pose3>(key),
                              marginals != nullptr ? marginals->marginalCovariance(key) : gtsam::Z_6x6};
  }

  std::string PoseWithCovariance::to_str() const
  {
    return to_str(pose_).append("\n").append(to_str(cov_));
  }

  std::string PoseWithCovariance::to_str(const gtsam::Pose3 &pose)
  {
    return to_row_str((gtsam::Vector{6} << pose.rotation().xyz(), pose.translation()).finished());
  }

  std::string PoseWithCovariance::to_str(const gtsam::Matrix6 &cov)
  {
    return to_lower_str(cov);
  }

  std::string PoseWithCovariance::to_eigenvalues_str(const gtsam::Matrix6 &cov)
  {
    Eigen::EigenSolver<gtsam::Matrix6> es(cov);
    gtsam::Vector6 evs{es.eigenvalues().real()};
    std::sort(evs.data(), evs.data() + evs.size(), std::greater<>());
    return to_row_str(evs);
  }

  std::string PoseWithCovariance::to_stddev_str(const gtsam::Matrix6 &cov)
  {
    return to_row_str(cov.diagonal().array().sqrt());
  }


  PointWithCovariance PointWithCovariance::Extract(const gtsam::Values &result,
                                                   const gtsam::Marginals *marginals,
                                                   gtsam::Key key)
  {
    return PointWithCovariance{key, result.at<gtsam::Point3>(key),
                               marginals != nullptr ? marginals->marginalCovariance(key) : gtsam::Z_3x3};
  }

  PointWithCovariance::FourPoints PointWithCovariance::Extract4(const gtsam::Values &result,
                                                                const gtsam::Marginals *marginals,
                                                                gtsam::Key marker_key)
  {
    return std::array<PointWithCovariance, 4>{
      Extract(result, marginals, CornerModel::corner_key(marker_key, 0)),
      Extract(result, marginals, CornerModel::corner_key(marker_key, 1)),
      Extract(result, marginals, CornerModel::corner_key(marker_key, 2)),
      Extract(result, marginals, CornerModel::corner_key(marker_key, 3)),
    };
  }


  std::string PointWithCovariance::to_str() const
  {
    return to_str(point_).append("\n").append(to_str(cov_));
  }

  std::string PointWithCovariance::to_str(const gtsam::Point3 &point)
  {
    return to_row_str(point);
  }

  std::string PointWithCovariance::to_str(const gtsam::Matrix3 &cov)
  {
    return to_lower_str(cov);
  }

  std::string PointWithCovariance::to_eigenvalues_str(const gtsam::Matrix3 &cov)
  {
    Eigen::EigenSolver<gtsam::Matrix3> es(cov);
    gtsam::Vector3 evs{es.eigenvalues().real()};
    std::sort(evs.data(), evs.data() + evs.size(), std::greater<>());
    return to_row_str(evs);
  }

  std::string PointWithCovariance::to_stddev_str(const gtsam::Matrix3 &cov)
  {
    return to_row_str(cov.diagonal().array().sqrt());
  }

}
