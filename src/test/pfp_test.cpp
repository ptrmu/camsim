

#include "catch2/catch.hpp"

#include <gtsam/base/numericalDerivative.h>
#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "../pfp/pfp_run.hpp"

using namespace gtsam;

namespace camsim
{
#if 0
  TEST_CASE("pfp_test - pfp_simple")
  {
    REQUIRE(pfp_simple() == 0);
  }

  TEST_CASE("pfp_test - pfp_simplest")
  {
    REQUIRE(pfp_simplest() == 0);
  }

  TEST_CASE("pfp_test - pfp_duplicate_prior()")
  {
    REQUIRE(pfp_duplicate_prior() == 0);
  }

  TEST_CASE("pfp_test - odometry_example_3d")
  {
    REQUIRE(odometry_example_3d() == 0);
  }

  TEST_CASE("pfp_test - pfp_pose_unit_test()")
  {
    REQUIRE(pfp_pose_unit_test() == 0);
  }

  TEST_CASE("pfp_test - pfp_marker_pose_from_corners()")
  {
    REQUIRE(pfp_marker_pose_from_corners() == 0);
  }

  TEST_CASE("pfp_test - pfp_gspnp()")
  {
    REQUIRE(pfp_gspnp() == 0);
  }
#endif


  const double degree = M_PI / 180;

  // Return a covariance matrix for type T with non-zero values for every element.
  // Use sigma_values^2 on the diagonal and fill in non-diagonal entries with
  // correlation coefficient of 1. Note: a covariance matrix for T has the same
  // dimensions as a Jacobian for T, the returned matrix is not a Jacobian.
  template<class T>
  typename T::Jacobian FullCovarianceFromSigmas(
    const typename T::TangentVector &sigmas)
  {
    return sigmas * sigmas.transpose();
  }

  // Return a covariance matrix with one non-zero element on the diagonal.
  template<class T>
  typename T::Jacobian SingleVariableCovarianceFromSigma(int idx, double sigma)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  // Return a covariance matrix with two non-zero elements on the diagonal and
  // a correlation between the two variables of 1.0
  template<class T>
  typename T::Jacobian TwoVariableCovarianceFromSigmas(int idx0, int idx1, double sigma0, double sigma1)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }

  template<class T>
  class TransformCovariance
  {
  private:
    typename T::Jacobian adjointMap_;

  public:
    TransformCovariance(const T &X) :
      adjointMap_(X.AdjointMap())
    {}

    typename T::Jacobian operator()(const typename T::Jacobian &covariance)
    { return adjointMap_ * covariance * adjointMap_.transpose(); }
  };

  TEST_CASE("pfp_test - covariance3_transform", "[.][all]")
  {
    // rotate
    {
      auto cov = FullCovarianceFromSigmas<Pose2>({0.1, 0.3, 0.7});
      auto transformed = TransformCovariance<Pose2>{{0., 0., 90 * degree}}(cov);
      // interchange x and y axes
      REQUIRE(assert_equal(
        Vector3{cov(1, 1), cov(0, 0), cov(2, 2)},
        Vector3{transformed.diagonal()}));
      REQUIRE(assert_equal(
        Vector3{-cov(1, 0), -cov(2, 1), cov(2, 0)},
        Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
    }

    // translate along x with uncertainty in x
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose2>(0, 0.1);
      auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
      // No change
      REQUIRE(assert_equal(cov, transformed));
    }

    // translate along x with uncertainty in y
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose2>(1, 0.1);
      auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
      // No change
      REQUIRE(assert_equal(cov, transformed));
    }

    // translate along x with uncertainty in theta
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose2>(2, 0.1);
      auto transformed = TransformCovariance<Pose2>{{20., 0., 0.}}(cov);
      REQUIRE(assert_equal(
        Vector3{0., 0.1 * 0.1 * 20. * 20., 0.1 * 0.1},
        Vector3{transformed.diagonal()}));
      REQUIRE(assert_equal(
        Vector3{0., 0., -0.1 * 0.1 * 20.},
        Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
    }

    // rotate and translate along x with uncertainty in x
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose2>(0, 0.1);
      auto transformed = TransformCovariance<Pose2>{{20., 0., 90 * degree}}(cov);
      // interchange x and y axes
      REQUIRE(assert_equal(
        Vector3{cov(1, 1), cov(0, 0), cov(2, 2)},
        Vector3{transformed.diagonal()}));
      REQUIRE(assert_equal(
        Vector3{Z_3x1},
        Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
    }

    // rotate and translate along x with uncertainty in theta
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose2>(2, 0.1);
      auto transformed = TransformCovariance<Pose2>{{20., 0., 90 * degree}}(cov);
      REQUIRE(assert_equal(
        Vector3{0., 0.1 * 0.1 * 20. * 20., 0.1 * 0.1},
        Vector3{transformed.diagonal()}));
      REQUIRE(assert_equal(
        Vector3{0., 0., -0.1 * 0.1 * 20.},
        Vector3{transformed(1, 0), transformed(2, 0), transformed(2, 1)}));
    }
  }

  TEST_CASE("pfp_test - covariance6_map_to_2d", "[.][all]")
  {
    Vector3 s2{0.1, 0.3, 0.7};
    Pose2 p2{1.1, 1.5, 31. * degree};
    auto cov2 = FullCovarianceFromSigmas<Pose2>(s2);
    auto transformed2 = TransformCovariance<Pose2>{p2}(cov2);

    auto match_cov3_to_cov2 = [&](int spatial_axis0, int spatial_axis1, int r_axis,
                                  const Pose2::Jacobian &cov2, const Pose3::Jacobian &cov3) -> void
    {
      REQUIRE(assert_equal(
        Vector3{cov2.diagonal()},
        Vector3{cov3(spatial_axis0, spatial_axis0), cov3(spatial_axis1, spatial_axis1), cov3(r_axis, r_axis)}));
      REQUIRE(assert_equal(
        Vector3{cov2(1, 0), cov2(2, 0), cov2(2, 1)},
        Vector3{cov3(spatial_axis1, spatial_axis0), cov3(r_axis, spatial_axis0), cov3(r_axis, spatial_axis1)}));
    };

    // rotate around x axis
    {
      auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << s2(2), 0., 0., 0., s2(0), s2(1)).finished());
      auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(p2.theta(), 0., 0.), {0., p2.x(), p2.y()}}}(cov3);
      match_cov3_to_cov2(4, 5, 0, transformed2, transformed3);
    }

    // rotate around y axis
    {
      auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., s2(2), 0., s2(1), 0., s2(0)).finished());
      auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., p2.theta(), 0.), {p2.y(), 0., p2.x()}}}(cov3);
      match_cov3_to_cov2(5, 3, 1, transformed2, transformed3);
    }

    // rotate around z axis
    {
      auto cov3 = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0., 0., s2(2), s2(0), s2(1), 0.).finished());
      auto transformed3 = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0., p2.theta()), {p2.x(), p2.y(), 0.}}}(cov3);
      match_cov3_to_cov2(3, 4, 2, transformed2, transformed3);
    }
  }

  TEST_CASE("pfp_test - covariance6_transform", "[.][all]")
  {
    // rotate 90 around z axis and then 90 around y axis
    {
#if 0 // this no longer works with new definition of Adjoint
      auto cov = FullCovarianceFromSigmas<Pose3>((Vector6{} << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1).finished());
      auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 90 * degree, 90 * degree), {}}}(cov);
      // x from y, y from z, z from x
      REQUIRE(assert_equal(
        (Vector6{} << cov(1, 1), cov(2, 2), cov(0, 0), cov(4, 4), cov(5, 5), cov(3, 3)).finished(),
        Vector6{transformed.diagonal()}));
      // Both the x and z axes are pointing in the negative direction.
      REQUIRE(assert_equal(
        (Vector5{} << -cov(2, 1), cov(0, 1), cov(4, 1), -cov(5, 1), cov(3, 1)).finished(),
        (Vector5{} << transformed(1, 0), transformed(2, 0), transformed(3, 0),
          transformed(4, 0), transformed(5, 0)).finished()));
#endif
    }

    // translate along the x axis with uncertainty in roty and rotz
    {
      auto cov = TwoVariableCovarianceFromSigmas<Pose3>(1, 2, 0.7, 0.3);
      auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(0., 0., 0.), {20., 0., 0.}}}(cov);
      // The uncertainty in roty and rotz causes off-diagonal covariances
      REQUIRE(assert_equal(0.7 * 0.7 * 20., transformed(5, 1)));
      REQUIRE(assert_equal(0.7 * 0.7 * 20. * 20., transformed(5, 5)));
      REQUIRE(assert_equal(-0.3 * 0.3 * 20., transformed(4, 2)));
      REQUIRE(assert_equal(0.3 * 0.3 * 20. * 20., transformed(4, 4)));
      REQUIRE(assert_equal(-0.3 * 0.7 * 20., transformed(4, 1)));
      REQUIRE(assert_equal(0.3 * 0.7 * 20., transformed(5, 2)));
      REQUIRE(assert_equal(-0.3 * 0.7 * 20. * 20., transformed(5, 4)));
    }

    // rotate around x axis and translate along the x axis with uncertainty in rotx
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose3>(0, 0.1);
      auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0., 0.), {20., 0., 0.}}}(cov);
      // No change
      REQUIRE(assert_equal(cov, transformed));
    }

    // rotate around x axis and translate along the x axis with uncertainty in roty
    {
      auto cov = SingleVariableCovarianceFromSigma<Pose3>(1, 0.1);
      auto transformed = TransformCovariance<Pose3>{{Rot3::RzRyRx(90 * degree, 0., 0.), {20., 0., 0.}}}(cov);
      // Uncertainty is spread to other dimensions.
      REQUIRE(assert_equal(
        (Vector6{} << 0., 0., 0.1 * 0.1, 0., 0.1 * 0.1 * 20. * 20., 0.).finished(),
        Vector6{transformed.diagonal()}));
    }
  }

  static double f(const Vector2& x) {
    assert(x.size() == 2);
    return sin(x(0)) + cos(x(1));
  }

  TEST_CASE("pfp_test - numericalGradient", "[.][all]")
  {
    Vector2 x(1, 1.1);

    Vector expected(2);
    expected << cos(x(0)), -sin(x(1));

    Vector actual = numericalGradient<Vector2>(f, x);

    REQUIRE(assert_equal(expected, actual, 1e-5));
  }

}
