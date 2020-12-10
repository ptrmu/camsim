#ifndef FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP
#define FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP

#include <Eigen/Geometry>

namespace fvlam
{

  class Translate3
  {
  public:
    using Derived = Eigen::Vector3d;
    using MuVector = Eigen::Matrix<double, Derived::MaxRowsAtCompileTime, 1>;

  private:
    Derived t_{Derived::Zero()};

  public:
    Translate3() = default;

    Translate3(const Derived &t) :
      t_(t)
    {}

    Translate3(double x, double y, double z) :
      t_(x, y, z)
    {}

    const auto &t() const
    { return t_; }

    std::string to_string();

    Translate3 operator+(const Translate3 &other) const
    {
      return Translate3(t_ + other.t_);
    }
  };

  class Translate3Covariance
  {
  public:
    using Derived = Eigen::Vector3d;
    using MuVector = Eigen::Matrix<double, 6, 1>;

  private:
    Derived cov_{Derived::Zero()};

  public:
    Translate3Covariance() = default;

    const auto &cov() const
    { return cov_; }

    std::string to_string();
  };

  class Translate3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Translate3::MuVector::MaxSizeAtCompileTime +
                                           Translate3Covariance::MuVector::MaxSizeAtCompileTime, 1>;

  private:
    bool is_valid_{false};
    Translate3 t_{};
    Translate3Covariance cov_{};

  public:
    Translate3WithCovariance() = default;

    auto is_valid() const
    { return is_valid_; }

    const auto &t() const
    { return t_; }

    const auto &cov() const
    { return cov_; }

    std::string to_string();
  };

  class Rotate3
  {
  public:
    using Derived = Eigen::Quaterniond;
    using MuVector = Eigen::Matrix<double, 3, 1>;

  private:
    Derived q_{Derived::Identity()};

  public:
    Rotate3() = default;

    Rotate3(const Derived &q) :
      q_(q)
    {}

    inline Rotate3(const double &w, const double &x, const double &y, const double &z) :
      q_(w, x, y, z)
    {}

    static Rotate3 Rx(double x)
    { return Rotate3{Derived{Eigen::AngleAxisd{x, Eigen::Vector3d::UnitX()}}}; }

    static Rotate3 Ry(double y)
    { return Rotate3{Derived{Eigen::AngleAxisd{y, Eigen::Vector3d::UnitY()}}}; }

    static Rotate3 Rz(double z)
    { return Rotate3{Derived{Eigen::AngleAxisd{z, Eigen::Vector3d::UnitZ()}}}; }

    static Rotate3 RzRyRx(double x, double y, double z)
    {
      return Rotate3{Derived{Eigen::AngleAxisd{z, Eigen::Vector3d::UnitZ()}} *
                     Derived{Eigen::AngleAxisd{y, Eigen::Vector3d::UnitY()}} *
                     Derived{Eigen::AngleAxisd{x, Eigen::Vector3d::UnitX()}}};
    }

    static Rotate3 Ypr(double y, double p, double r)
    { return RzRyRx(r, p, y); }

    const auto &q() const
    { return q_; }

    const Eigen::Matrix3d matrix() const
    { return q_.toRotationMatrix(); }

    Eigen::Vector3d xyz() const;

    std::string to_string();

    Rotate3 inverse()
    {
      return Rotate3(q_.inverse());
    }

    Rotate3 operator*(const Rotate3 &other) const
    {
      return Rotate3(q_ * other.q_);
    }

    Translate3 operator*(const Translate3 &other) const
    {
      return Translate3(q_ * other.t());
    }
  };

  class Transform3
  {
  public:
    using MuVector = Eigen::Matrix<double,
      Rotate3::MuVector::MaxSizeAtCompileTime +
      Translate3::MuVector::MaxSizeAtCompileTime, 1>;

  private:
    Rotate3 r_{};
    Translate3 t_{};

  public:
    Transform3() = default;

    Transform3(const Rotate3 &r, const Translate3 &p) :
      r_(r), t_(p)
    {}

    const auto &r() const
    { return r_; }

    const auto &p() const
    { return t_; }

    std::string to_string();

    Transform3 operator*(const Transform3 &other) const
    {
      return Transform3(r_ * other.r_, t_ + r_ * other.t_);
    }
  };

  class Transform3Covariance
  {
  public:
    using Derived = Eigen::Matrix<double,
      Transform3::MuVector::MaxSizeAtCompileTime,
      Transform3::MuVector::MaxSizeAtCompileTime>;
    using MuVector = Eigen::Matrix<double, 21, 1>;

  private:
    Derived cov_{Derived::Zero()};

  public:
    Transform3Covariance() = default;

    explicit Transform3Covariance(Derived &cov) :
      cov_(cov)
    {}

    const auto &cov() const
    { return cov_; }

    std::string to_string();
  };

  class Transform3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Transform3::MuVector::MaxRowsAtCompileTime +
                                           Transform3Covariance::MuVector::MaxRowsAtCompileTime, 1>;
  private:
    bool is_valid_{false};
    Transform3 tf_{};
    Transform3Covariance cov_{};

  public:
    Transform3WithCovariance() = default;

    Transform3WithCovariance(const Transform3 &tf, const Transform3Covariance &cov) :
      is_valid_(true), tf_(tf), cov_(cov)
    {}

    auto is_valid() const
    { return is_valid_; }

    const auto &tf() const
    { return tf_; }

    const auto &cov() const
    { return cov_; }

    std::string to_string();
  };


  template<typename T>
  Translate3 toTranslate3(const T &other); //
  template<typename T>
  Translate3Covariance toTranslate3Covariance(const T &other); //
  template<typename T>
  Rotate3 toRotate3(const T &other); //
  template<typename T>
  Transform3 toTransform3(const T &other); //
  template<typename T>
  Transform3Covariance toTransform3Covariance(const T &other); //
  template<typename T>
  T fromTranslate3(const Translate3 &other); //
  template<typename T>
  T fromTranslate3Covariance(const Translate3Covariance &other); //
  template<typename T>
  T fromRotate3(const Rotate3 &other); //
  template<typename T>
  T fromTransform3(const Transform3 &other); //
  template<typename T>
  T fromTransform3Covariance(const Transform3Covariance &other); //

}

#endif // FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP
