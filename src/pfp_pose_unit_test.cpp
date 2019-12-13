
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

  template<class TPose>
  static typename TPose::Jacobian generate_full_covariance(
    std::array<double, TPose::dimension> sigma_values)
  {
    typename TPose::TangentVector sigmas(&sigma_values.front());
    return typename TPose::Jacobian{sigmas * sigmas.transpose()};
  }

  template<class TPose>
  static typename TPose::Jacobian generate_one_variable_covariance(int idx, double sigma)
  {
    typename TPose::Jacobian cov = TPose::Jacobian::Zero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  template<class TPose>
  static typename TPose::Jacobian generate_two_variable_covariance(int idx0, int idx1, double sigma0, double sigma1)
  {
    typename TPose::Jacobian cov = TPose::Jacobian::Zero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }

  template<class TPose>
  static typename TPose::Jacobian transform_covariance(
    const TPose &wTb,
    const typename TPose::Jacobian &cov)
  {
    // transform the covariance with the AdjointMap
    auto adjointMap = wTb.AdjointMap();
    return adjointMap * cov * adjointMap.transpose();
  }

  static typename Pose2::Jacobian rotate_only(
    const std::array<double, Pose2::Rotation::dimension> r,
    const typename Pose2::Jacobian &cov)
  {
    // Create a rotation only transform
    Pose2 wTb{Pose2::Rotation{r[0] * degree}, Pose2::Translation{}};
    return transform_covariance(wTb, cov);
  }

  static Pose2::Jacobian translate_only(
    const std::array<double, Pose2::Translation::dimension> t,
    const Pose2::Jacobian &cov)
  {
    // Create a translation only transform
    Pose2 wTb{Pose2::Rotation{}, Pose2::Translation{t[0], t[1]}};
    return transform_covariance(wTb, cov);
  }

  static Pose2::Jacobian rotate_translate(
    const std::array<double, Pose2::Rotation::dimension> r,
    const std::array<double, Pose2::Translation::dimension> t,
    const Pose2::Jacobian &cov)
  {
    // Create a translation only transform
    Pose2 wTb{Pose2::Rotation{r[0] * degree}, Pose2::Translation{t[0], t[1]}};
    return transform_covariance(wTb, cov);
  }

  static typename Pose3::Jacobian rotate_only(
    const std::array<double, Pose3::Rotation::dimension> r,
    const typename Pose3::Jacobian &cov)
  {
    // Create a rotation only transform
    Pose3 wTb{Pose3::Rotation::RzRyRx(r[0] * degree, r[1] * degree, r[2] * degree),
              Pose3::Translation{}};
    return transform_covariance(wTb, cov);
  }

  static Pose3::Jacobian translate_only(
    const std::array<double, Pose3::Translation::dimension> t,
    const Pose3::Jacobian &cov)
  {
    // Create a translation only transform
    Pose3 wTb{Pose3::Rotation{}, Pose3::Translation{t[0], t[1], t[2]}};
    return transform_covariance(wTb, cov);
  }

  static Pose3::Jacobian rotate_translate(
    const std::array<double, Pose3::Rotation::dimension> r,
    const std::array<double, Pose3::Translation::dimension> t,
    const Pose3::Jacobian &cov)
  {
    // Create a translation only transform
    Pose3 wTb{Pose3::Rotation::RzRyRx(r[0] * degree, r[1] * degree, r[2] * degree),
              Pose3::Translation{t[0], t[1], t[2]}};
    return transform_covariance(wTb, cov);
  }

  static void pfp_covariance3_transform_unit_test()
  {
    // rotate
    {
      auto cov = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
      auto cov_trans = rotate_only({90.}, cov);
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
      auto cov = generate_one_variable_covariance<Pose2>(0, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0., 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), 0., 1e-9);
    }

    // translate along x with uncertainty in y
    {
      auto cov = generate_one_variable_covariance<Pose2>(1, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0., 1e-9);

      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), 0., 1e-9);
    }

    // translate along x with uncertainty in theta
    {
      auto cov = generate_one_variable_covariance<Pose2>(2, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 1), -0.1 * 0.1 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1 * 20. * 20., 1e-9);
    }

    // rotate and translate along x with uncertainty in x
    {
      auto cov = generate_one_variable_covariance<Pose2>(0, 0.1);
      auto cov_trans = rotate_translate({90.}, {20., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), 0.1 * 0.1, 1e-9);
    }

    // rotate and translate along x with uncertainty in theta
    {
      auto cov = generate_one_variable_covariance<Pose2>(2, 0.1);
      auto cov_trans = rotate_translate({90.}, {20., 0.}, cov);
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
    auto cov2 = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
    auto cov2_trans = rotate_translate(r, t, cov2);

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
      auto cov3 = generate_full_covariance<Pose3>({s[2], 0., 0., 0., s[0], s[1]});
      auto cov3_trans = rotate_translate({r[0], 0., 0.}, {0., t[0], t[1]}, cov3);
      match_cov3_to_cov2(0, 4, 5, cov2_trans, cov3_trans);
    }

    // rotate around y axis
    {
      auto cov3 = generate_full_covariance<Pose3>({0., s[2], 0., s[1], 0., s[0]});
      auto cov3_trans = rotate_translate({0., r[0], 0.}, {t[1], 0., t[0]}, cov3);
      match_cov3_to_cov2(1, 5, 3, cov2_trans, cov3_trans);
    }

    // rotate around z axis
    {
      auto cov3 = generate_full_covariance<Pose3>({0., 0., s[2], s[0], s[1], 0.});
      auto cov3_trans = rotate_translate({0., 0., r[0]}, {t[0], t[1], 0.}, cov3);
      match_cov3_to_cov2(2, 3, 4, cov2_trans, cov3_trans);
    }
  }

  static void pfp_covariance6_transform_unit_test()
  {
    // rotate 90 around z axis and then 90 around y axis
    {
      auto cov = generate_full_covariance<Pose3>({0.1, 0.2, 0.3, 0.5, 0.7, 1.1});
      auto cov_trans = rotate_only({0., 90., 90.}, cov);
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
      auto cov = generate_two_variable_covariance<Pose3>(1, 2, 0.7, 0.3);
      auto cov_trans = translate_only({20., 0., 0.}, cov);
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
      auto cov = generate_one_variable_covariance<Pose3>(0, 0.1);
      auto cov_trans = rotate_translate({90., 0., 0.}, {20., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
    }

    // rotate around x axis and translate along the x axis with uncertainty in roty
    {
      auto cov = generate_one_variable_covariance<Pose3>(1, 0.1);
      auto cov_trans = rotate_translate({90., 0., 0.}, {20., 0., 0.}, cov);
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
