
#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
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
      return *this;
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
  template<typename FixedSizeVector>
  static std::string to_row_str(const FixedSizeVector &v)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (int r = 0; r < FixedSizeVector::MaxSizeAtCompileTime; r += 1) {
      if (r != 0) {
        ss << " ";
      }
      ss << nf(v(r));
    }
    return ss.str();
  }

  template<class FixedSizeMatrix>
  static std::string to_matrix_str(const FixedSizeMatrix &m, bool lower_only)
  {
    std::stringstream ss{};
    NumFmt nf(9, 3);
    for (int r = 0; r < m.rows(); r += 1) {
      if (r != 0) {
        ss << std::endl;
      }
      for (int c = 0; c <= (lower_only ? r : m.cols()); c += 1) {
        if (c != 0) {
          ss << " ";
        }
        ss << nf(m(r, c));
      }
    }
    return ss.str();
  }

  Eigen::Vector3d Rotate3::xyz() const
  {
    const auto X = q_.toRotationMatrix();
    const double x = -atan2(-X(2, 1), X(2, 2));

#if 0
    const Eigen::Matrix3d Y = X * Rx(-x).q().toRotationMatrix();
    const double y = -atan2(Y(2, 0), Y(2, 2));

    const Eigen::Matrix3d Z = Y * Ry(-y).q().toRotationMatrix();
    const double z = -atan2(-Z(1, 0), Z(1, 1));
#else
    const auto qy = q_ * Rx(-x).q();
    const auto Y = qy.toRotationMatrix();
    const double y = -atan2(Y(2, 0), Y(2, 2));

    const auto qz = qy * Ry(-y).q();
    const auto Z = qz.toRotationMatrix();
    const double z = -atan2(-Z(1, 0), Z(1, 1));
#endif
    return Eigen::Vector3d(x, y, z);
  }

  std::string Translate3::to_string() const
  {
    return to_row_str(t_);
  }

  std::string Rotate3::to_string() const
  {
    return to_row_str(xyz());
  }

  std::string Transform3::to_string() const
  {
    return r_.to_string() + " " + t_.to_string();
  }

  std::string Transform3Covariance::to_string() const
  {
    return to_matrix_str(cov_, true);
  }

  std::string Transform3WithCovariance::to_string() const
  {
    return tf_.to_string()+"\n"+cov_.to_string();
  }
}