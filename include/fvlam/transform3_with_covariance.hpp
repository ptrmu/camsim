#ifndef FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP
#define FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <Eigen/Geometry>

namespace fvlam
{

// ==============================================================================
// Translate3 class
// ==============================================================================

  class Translate3
  {
  public:
    using MuVector = Eigen::Vector3d;
    using TangentVector = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    MuVector t_{MuVector::Zero()};

  public:
    Translate3() = default;

    explicit Translate3(MuVector t) :
      t_(std::move(t))
    {}

    Translate3(double x, double y, double z) :
      t_(x, y, z)
    {}

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return t_; }

    template<typename T>
    static Translate3 from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;

    /// Exponential map at identity - create a translation from canonical coordinates \f$ [T_x,T_y,T_z] \f$
    static Translate3 Expmap(const TangentVector &x)
    { return Translate3(x); }

    /// Log map at identity - return the canonical coordinates \f$ [T_x,T_y,T_z] \f$ of this translation
    static TangentVector Logmap(const Translate3 &translate3)
    { return translate3.t_; }

    Translate3 cross(const Translate3 &v) const
    { return Translate3{t_.cross(v.t_)}; }

    Translate3 operator+(const Translate3 &other) const
    {
      return Translate3(t_ + other.t_);
    }
  };

// ==============================================================================
// Translate3WithCovariance class
// ==============================================================================

  class Translate3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Translate3::MuVector::MaxSizeAtCompileTime +
                                           Translate3::CovarianceMatrix::MaxSizeAtCompileTime, 1>;

  private:
    bool is_valid_{false};
    Translate3 t_{};
    Translate3::CovarianceMatrix cov_{};

  public:
    Translate3WithCovariance() = default;

    auto is_valid() const
    { return is_valid_; }

    const auto &t() const
    { return t_; }

    const auto &cov() const
    { return cov_; }

    template<typename T>
    static Translate3WithCovariance from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;
  };

// ==============================================================================
// Rotate3 class
// ==============================================================================

  class Rotate3
  {
  public:
    using MuVector = Eigen::Matrix<double, 3, 1>;
    using TangentVector = Eigen::Matrix<double, 3, 1>;
    using RotationMatrix = Eigen::Matrix<double, 3, 3>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    using Derived = Eigen::Quaterniond;
    Derived q_{Derived::Identity()};

  public:
    Rotate3() = default;

    explicit Rotate3(const Derived &q) :
      q_(q)
    {}

    explicit Rotate3(const RotationMatrix &rotation_matrix) :
      q_(rotation_matrix)
    {}

    explicit Rotate3(const double &w, const double &x, const double &y, const double &z) :
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

    RotationMatrix rotation_matrix() const
    { return q_.toRotationMatrix(); }

    MuVector xyz() const;

    MuVector mu() const
    { return xyz(); }

    template<typename T>
    static Rotate3 from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;

    Rotate3 inverse() const
    {
      return Rotate3(q_.inverse());
    }

    /// Exponential map at identity - create a rotation from canonical coordinates \f$ [R_x,R_y,R_z] \f$
    static Rotate3 Expmap(const TangentVector &x);

    /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z] \f$ of this rotation
    static TangentVector Logmap(const Rotate3 &rotate3);

    Rotate3 operator*(const Rotate3 &other) const
    {
      return Rotate3(q_ * other.q_);
    }

    Translate3 operator*(const Translate3 &other) const
    {
      return Translate3(q_ * other.t());
    }
  };

// ==============================================================================
// Transform3 class
// ==============================================================================

  class Transform3
  {
  public:
    using MuVector = Eigen::Matrix<double,
      Rotate3::MuVector::MaxSizeAtCompileTime +
      Translate3::MuVector::MaxSizeAtCompileTime, 1>;
    using TangentVector = Eigen::Matrix<double,
      Rotate3::MuVector::MaxSizeAtCompileTime +
      Translate3::MuVector::MaxSizeAtCompileTime, 1>;
    using CovarianceMatrix = Eigen::Matrix<double, MuVector::MaxSizeAtCompileTime, MuVector::MaxSizeAtCompileTime>;

  private:
    Rotate3 r_{};
    Translate3 t_{};

  public:
    Transform3() = default;

    Transform3(Rotate3 r, Translate3 t) :
      r_(std::move(r)), t_(std::move(t))
    {}

    explicit Transform3(const MuVector &mu) :
      r_(Rotate3::RzRyRx(mu(0), mu(1), mu(2))),
      t_(Translate3(mu(3), mu(4), mu(5)))
    {}

    const auto &r() const
    { return r_; }

    const auto &t() const
    { return t_; }

    MuVector mu() const
    { return (MuVector() << r_.mu(), t_.mu()).finished(); }

    template<typename T>
    static Transform3 from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;

    static std::string to_cov_string(const CovarianceMatrix &cov);

    Transform3 inverse() const
    {
      auto qi = r_.q().inverse();
      return Transform3(Rotate3(qi), Translate3(qi * -t_.t()));
    }

    /// Exponential map at identity - create a transform from canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$
    static Transform3 Expmap(const TangentVector &x);

    /// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of this transform
    static TangentVector Logmap(const Transform3 &transform3);

    Transform3 operator*(const Transform3 &other) const
    {
      return Transform3(r_ * other.r_, t_ + r_ * other.t_);
    }
  };

// ==============================================================================
// Transform3WithCovariance class
// ==============================================================================

  class Transform3WithCovariance
  {
  public:
    using MuVector = Eigen::Matrix<double, Transform3::MuVector::MaxSizeAtCompileTime +
                                           Transform3::CovarianceMatrix::MaxSizeAtCompileTime, 1>;

  private:
    bool is_valid_{false};
    Transform3 tf_{};
    Transform3::CovarianceMatrix cov_{};

  public:
    Transform3WithCovariance() = default;

    Transform3WithCovariance(Transform3 tf, Transform3::CovarianceMatrix cov) :
      is_valid_(true), tf_(std::move(tf)), cov_(std::move(cov))
    {}

    auto is_valid() const
    { return is_valid_; }

    const auto &tf() const
    { return tf_; }

    const auto &cov() const
    { return cov_; }

    template<typename T>
    static Transform3WithCovariance from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;
  };
}

#endif // FVLAM_TRANSFORM3_WITH_COVARIANCE_HPP
