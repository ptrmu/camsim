
#include "pfp_run.hpp"

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "pose_with_covariance.hpp"

using namespace gtsam;

#define EXPECT_DOUBLES_EQUAL(expected, actual, threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (!std::isfinite(actualTemp) || !std::isfinite(expectedTemp) || std::fabs ((expectedTemp)-(actualTemp)) > threshold) \
    { std::cout << "Failure file:" << __FILE__ << " line:" << __LINE__ << std::endl; } }

namespace camsim
{
  const double degree = M_PI / 180;

  // Create a covariance matrix for type T. Use sigma_values^2 on the diagonal
  // and fill in non-diagonal entries with correlation coefficient of 1. Note:
  // a covariance matrix for T has the same dimensions as a Jacobian for T.
  template<class T>
  typename T::Jacobian GenerateFullCovariance(
    std::array<double, T::dimension> sigma_values)
  {
    typename T::TangentVector sigmas(&sigma_values.front());
    return typename T::Jacobian{sigmas * sigmas.transpose()};
  }

  // Create a covariance matrix with one non-zero element on the diagonal.
  template<class T>
  typename T::Jacobian GenerateOneVariableCovariance(int idx, double sigma)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  // Create a covariance matrix with two non-zero elements on the diagonal with
  // a correlation of 1.0
  template<class T>
  typename T::Jacobian GenerateTwoVariableCovariance(int idx0, int idx1, double sigma0, double sigma1)
  {
    typename T::Jacobian cov = T::Jacobian::Zero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }

  // Overloaded function to create a Rot2 from one angle.
  Rot2 RotFromArray(const std::array<double, Rot2::dimension> &r)
  {
    return Rot2{r[0] * degree};
  }

  // Overloaded function to create a Rot3 from three angles.
  Rot3 RotFromArray(const std::array<double, Rot3::dimension> &r)
  {
    return Rot3::RzRyRx(r[0] * degree, r[1] * degree, r[2] * degree);
  }

  // Transform a covariance matrix with a rotation and a translation
  template<class Pose>
  typename Pose::Jacobian RotateTranslate(
    std::array<double, Pose::Rotation::dimension> r,
    std::array<double, Pose::Translation::dimension> t,
    const typename Pose::Jacobian &cov)
  {
    // Construct a pose object
    typename Pose::Rotation rot{RotFromArray(r)};
    Pose wTb{rot, typename Pose::Translation{&t.front()}};

    // transform the covariance with the AdjointMap
    auto adjointMap = wTb.AdjointMap();
    return adjointMap * cov * adjointMap.transpose();
  }

  static void pfp_covariance3_transform_unit_test()
  {
    // rotate
    {
      auto cov = GenerateFullCovariance<Pose2>({0.1, 0.3, 0.7});
      auto cov_trans = RotateTranslate<Pose2>({{90.}}, {{}}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), cov(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), cov(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), cov(2, 2), 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), -cov(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), -cov(2, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), cov(2, 0), 1e-9);
    }

    // translate along x with uncertainty in x
    {
      auto cov = GenerateOneVariableCovariance<Pose2>(0, 0.1);
      auto cov_trans = RotateTranslate<Pose2>({{}}, {{20., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0., 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), 0., 1e-9);
    }

    // translate along x with uncertainty in y
    {
      auto cov = GenerateOneVariableCovariance<Pose2>(1, 0.1);
      auto cov_trans = RotateTranslate<Pose2>({{}}, {{20., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0., 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), 0., 1e-9);
    }

    // translate along x with uncertainty in theta
    {
      auto cov = GenerateOneVariableCovariance<Pose2>(2, 0.1);
      auto cov_trans = RotateTranslate<Pose2>({{}}, {{20., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), -0.1 * 0.1 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1 * 20. * 20., 1e-9);
    }

    // rotate and translate along x with uncertainty in x
    {
      auto cov = GenerateOneVariableCovariance<Pose2>(0, 0.1);
      auto cov_trans = RotateTranslate<Pose2>({{90.}}, {{20., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1, 1e-9);
    }

    // rotate and translate along x with uncertainty in theta
    {
      auto cov = GenerateOneVariableCovariance<Pose2>(2, 0.1);
      auto cov_trans = RotateTranslate<Pose2>({{90.}}, {{20., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1 * 20. * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0.1 * 0.1, 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), -0.1 * 0.1 * 20., 1e-9);
    }
  }

  static void pfp_covariance6_map_to_2d_unit_test()
  {
    std::array<double, Pose2::dimension> s{{0.1, 0.3, 0.7}};
    std::array<double, Pose2::Rotation::dimension> r{{31.}};
    std::array<double, Pose2::Translation::dimension> t{{1.1, 1.5}};
    auto cov2 = GenerateFullCovariance<Pose2>({{0.1, 0.3, 0.7}});
    auto cov2_trans = RotateTranslate<Pose2>(r, t, cov2);

    auto match_cov3_to_cov2 = [&](int r_axis, int spatial_axis0, int spatial_axis1,
                                  const Pose2::Jacobian &cov2, const Pose3::Jacobian &cov3) -> void
    {
      EXPECT_DOUBLES_EQUAL(cov2(0, 0), cov3(spatial_axis0, spatial_axis0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov2(1, 1), cov3(spatial_axis1, spatial_axis1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov2(2, 2), cov3(r_axis, r_axis), 1e-9);

      EXPECT_DOUBLES_EQUAL(cov2(1, 0), cov3(spatial_axis1, spatial_axis0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov2(2, 1), cov3(r_axis, spatial_axis1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov2(2, 0), cov3(r_axis, spatial_axis0), 1e-9);
    };

    // rotate around x axis
    {
      auto cov3 = GenerateFullCovariance<Pose3>({{s[2], 0., 0., 0., s[0], s[1]}});
      auto cov3_trans = RotateTranslate<Pose3>({{r[0], 0., 0.}}, {{0., t[0], t[1]}}, cov3);
      match_cov3_to_cov2(0, 4, 5, cov2_trans, cov3_trans);
    }

    // rotate around y axis
    {
      auto cov3 = GenerateFullCovariance<Pose3>({{0., s[2], 0., s[1], 0., s[0]}});
      auto cov3_trans = RotateTranslate<Pose3>({{0., r[0], 0.}}, {{t[1], 0., t[0]}}, cov3);
      match_cov3_to_cov2(1, 5, 3, cov2_trans, cov3_trans);
    }

    // rotate around z axis
    {
      auto cov3 = GenerateFullCovariance<Pose3>({{0., 0., s[2], s[0], s[1], 0.}});
      auto cov3_trans = RotateTranslate<Pose3>({{0., 0., r[0]}}, {{t[0], t[1], 0.}}, cov3);
      match_cov3_to_cov2(2, 3, 4, cov2_trans, cov3_trans);
    }
  }

  static void pfp_covariance6_transform_unit_test()
  {
    // rotate 90 around z axis and then 90 around y axis
    {
      auto cov = GenerateFullCovariance<Pose3>({{0.1, 0.2, 0.3, 0.5, 0.7, 1.1}});
      auto cov_trans = RotateTranslate<Pose3>({{0., 90., 90.}}, {{}}, cov);
      // x from y, y from z, z from x
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), cov(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), cov(2, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), cov(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(3, 3), cov(4, 4), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), cov(5, 5), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), cov(3, 3), 1e-9);

      // Both the x and z axes are pointing in the negative direction.
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), -cov(2, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), cov(0, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(3, 0), cov(4, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 0), -cov(5, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 0), cov(3, 1), 1e-9);
    }

    // translate along the x axis with uncertainty in roty and rotz
    {
      auto cov = GenerateTwoVariableCovariance<Pose3>(1, 2, 0.7, 0.3);
      auto cov_trans = RotateTranslate<Pose3>({{}}, {{20., 0., 0.}}, cov);
      // The variance in roty and rotz causes off-diagonal covariances
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 1), 0.7 * 0.7 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 0.7 * 0.7 * 20. * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 2), -0.3 * 0.3 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0.3 * 0.3 * 20. * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 1), -0.3 * 0.7 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 2), 0.3 * 0.7 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 4), -0.3 * 0.7 * 20. * 20., 1e-9);
    }

    // rotate around x axis and translate along the x axis with uncertainty in rotx
    {
      auto cov = GenerateOneVariableCovariance<Pose3>(0, 0.1);
      auto cov_trans = RotateTranslate<Pose3>({{90., 0., 0.}}, {{20., 0., 0.}}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
    }

    // rotate around x axis and translate along the x axis with uncertainty in roty
    {
      auto cov = GenerateOneVariableCovariance<Pose3>(1, 0.1);
      auto cov_trans = RotateTranslate<Pose3>({{90., 0., 0.}}, {{20., 0., 0.}}, cov);
      // interchange the y and z axes.
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 2), -0.1 * 0.1 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0.1 * 0.1 * 20. * 20., 1e-9);
    }
  }

  void pfp_pose_unit_test()
  {
    pfp_covariance3_transform_unit_test();
    pfp_covariance6_map_to_2d_unit_test();
    pfp_covariance6_transform_unit_test();
  }
}
