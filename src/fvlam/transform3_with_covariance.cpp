
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


  Eigen::Vector3d Rotate3::xyz() const
  {
    auto A = matrix();

    const double x = -atan2(-A(2, 1), A(2, 2));
    const auto Qx = Rx(-x).matrix();
    const Eigen::Matrix3d B = A * Qx;

    const double y = -atan2(B(2, 0), B(2, 2));
    const auto Qy = Ry(-y).matrix();
    const Eigen::Matrix3d C = B * Qy;

    const double z = -atan2(-C(1, 0), C(1, 1));

    return Eigen::Vector3d(x, y, z);
  }

  std::string Rotate3::to_string()
  {
    return to_row_str(xyz());
  }
}