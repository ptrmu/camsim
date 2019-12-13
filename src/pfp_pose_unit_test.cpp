
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

//  template<>
//  typename Pose3::Jacobian transform_covariance(
//    const Pose3 &wTb,
//    const typename Pose3::Jacobian &cov)
//  {
//    // transform the covariance with the AdjointMap
//    auto adjointMap = wTb.AdjointMap();
//    Pose3::Jacobian cov_trans = adjointMap * cov * adjointMap.transpose();
//
//    std::cout << PoseWithCovariance::to_str(wTb) << std::endl;
//    std::cout << PoseWithCovariance::to_str(cov) << std::endl;
//    std::cout << PoseWithCovariance::to_str(cov_trans) << std::endl << std::endl;
//
//    return cov_trans;
//  }
//
//  template<>
//  typename Pose2::Jacobian transform_covariance(
//    const Pose2 &wTb,
//    const typename Pose2::Jacobian &cov)
//  {
//    // transform the covariance with the AdjointMap
//    auto adjointMap = wTb.AdjointMap();
//    Pose2::Jacobian cov_trans = adjointMap * cov * adjointMap.transpose();
//
//    std::cout << wTb.rotation().theta() << " " << wTb.translation() << std::endl;
//    std::cout << cov << std::endl;
//    std::cout << cov_trans << std::endl << std::endl;
//
//    return cov_trans;
//  }

  static typename Pose2::Jacobian rotate_only(
    const std::array<double, Pose2::Rotation::dimension> r,
    const typename Pose2::Jacobian &cov)
  {
    // Create a rotation only transform
    Pose2 wTb{Pose2::Rotation{r[0] * degree}, Pose2::Translation{}};
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

  static Pose2::Jacobian translate_only(
    const std::array<double, Pose2::Translation::dimension> t,
    const Pose2::Jacobian &cov)
  {
    // Create a translation only transform
    Pose2 wTb{Pose2::Rotation{}, Pose2::Translation{t[0], t[1]}};
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

  static Pose2::Jacobian rotate_translate(
    const std::array<double, Pose2::Rotation::dimension> r,
    const std::array<double, Pose2::Translation::dimension> t,
    const Pose2::Jacobian &cov)
  {
    // Create a translation only transform
    Pose2 wTb{Pose2::Rotation{r[0] * degree}, Pose2::Translation{t[0], t[1]}};
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

  static void pfp_covariance2_rotation_unit_test()
  {
    // rotate
    {
      auto cov = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
      auto cov_trans = rotate_only({90.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(cov(0, 0), cov_trans(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov(1, 1), cov_trans(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov(2, 2), cov_trans(2, 2), 1e-9);

      EXPECT_DOUBLES_EQUAL(-cov(1, 0), cov_trans(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov(2, 1), cov_trans(2, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov(2, 0), cov_trans(2, 1), 1e-9);
    }

    // translate along x with uncertainty in x
    {
      auto cov = generate_one_variable_covariance<Pose2>(0, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(0.1 * 0.1, cov_trans(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 2), 1e-9);

      EXPECT_DOUBLES_EQUAL(0., cov_trans(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 1), 1e-9);
    }

    // translate along x with uncertainty in y
    {
      auto cov = generate_one_variable_covariance<Pose2>(1, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(0., cov_trans(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0.1 * 0.1, cov_trans(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 2), 1e-9);

      EXPECT_DOUBLES_EQUAL(0., cov_trans(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 1), 1e-9);
    }

    // translate along x with uncertainty in theta
    {
      auto cov = generate_one_variable_covariance<Pose2>(2, 0.1);
      auto cov_trans = translate_only({20., 0.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(-0.1 * 0.1 * 20., cov_trans(2, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(0.1 * 0.1 * 20. * 20., cov_trans(1, 1), 1e-9);
    }

    // rotate and translate along x with uncertainty in x
    {
      auto cov = generate_one_variable_covariance<Pose2>(0, 0.1);
      auto cov_trans = rotate_translate({90.}, {20., 0.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(0., cov_trans(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0.1 * 0.1, cov_trans(1, 1), 1e-9);
    }

    // rotate and translate along x with uncertainty in theta
    {
      auto cov = generate_one_variable_covariance<Pose2>(2, 0.1);
      auto cov_trans = rotate_translate({90.}, {20., 0.}, cov);
      // interchange x and y axes
      EXPECT_DOUBLES_EQUAL(0., cov_trans(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0.1 * 0.1, cov_trans(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 2), 1e-9);

      EXPECT_DOUBLES_EQUAL(0., cov_trans(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(0., cov_trans(2, 1), 1e-9);
    }
  }

  static std::array<double, Rot3::dimension> to_rotation3(int the_axis, double r)
  {
    std::array<double, Rot3::dimension> rotation3{};
    rotation3[the_axis] = r;
    return rotation3;
  }

  static std::array<double, Point3::dimension> to_translation3(int the_axis, std::array<double, Point2::dimension> t)
  {
    int spatial_axis0 = the_axis > 0 ? 0 : 1;
    int spatial_axis1 = the_axis > 1 ? 1 : 2;

    std::array<double, Point3::dimension> translation3{};
    translation3[spatial_axis0] = t[0];
    translation3[spatial_axis1] = t[1];
    return translation3;
  }

  static std::array<double, Pose3::dimension> to_sigmas6(int the_axis, std::array<double, Pose2::dimension> sigmas3)
  {
    int spatial_axis0 = the_axis > 0 ? 3 : 4;
    int spatial_axis1 = the_axis > 1 ? 4 : 5;

    std::array<double, Pose3::dimension> sigmas6{};
    sigmas6[the_axis] = sigmas3[2];
    sigmas6[spatial_axis0] = sigmas3[0];
    sigmas6[spatial_axis1] = sigmas3[1];
    return sigmas6;
  }

  static int to_axis6(int the_axis, int pose2_axis)
  {
    if (pose2_axis == 2) {
      return the_axis;
    }
    int spatial_axis0 = the_axis > 0 ? 3 : 4;
    int spatial_axis1 = the_axis > 1 ? 4 : 5;
    return pose2_axis == 0 ? spatial_axis0 : spatial_axis1;
  }

  static void match_cov3_to_cov2(int the_axis, const Pose2::Jacobian &cov2, const Pose3::Jacobian &cov3)
  {
    int spatial_axis0 = the_axis > 0 ? 3 : 4;
    int spatial_axis1 = the_axis > 1 ? 4 : 5;

    EXPECT_DOUBLES_EQUAL(cov2(0, 0), cov3(spatial_axis0, spatial_axis0), 1e-9);
    EXPECT_DOUBLES_EQUAL(cov2(1, 1), cov3(spatial_axis1, spatial_axis1), 1e-9);
    EXPECT_DOUBLES_EQUAL(cov2(2, 2), cov3(the_axis, the_axis), 1e-9);

    EXPECT_DOUBLES_EQUAL(cov2(1, 0), cov3(spatial_axis1, spatial_axis0), 1e-9);
    EXPECT_DOUBLES_EQUAL(cov2(2, 1), cov3(the_axis, spatial_axis1), 1e-9);
    EXPECT_DOUBLES_EQUAL(cov2(2, 0), cov3(the_axis, spatial_axis0), 1e-9);
  }

  static void pfp_covariance_one_axis_unit_test(int the_axis)
  {
    // rotate
    {
      auto cov2 = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
      auto cov3 = generate_full_covariance<Pose3>(to_sigmas6(the_axis, {0.1, 0.3, 0.7}));
      auto cov2_trans = rotate_only({90.}, cov2);
      auto cov3_trans = rotate_only(to_rotation3(the_axis, 90.), cov3);
      match_cov3_to_cov2(the_axis, cov2, cov3);
    }

    // translate along x
    {
      auto cov2 = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
      auto cov3 = generate_full_covariance<Pose3>(to_sigmas6(the_axis, {0.1, 0.3, 0.7}));
      auto cov2_trans = translate_only({20., 0.}, cov2);
      auto cov3_trans = translate_only(to_translation3(the_axis, {20., 0.}), cov3);
      match_cov3_to_cov2(the_axis, cov2, cov3);
    }

    // rotate and translate along x with uncertainty in x
    {
      auto cov2 = generate_full_covariance<Pose2>({0.1, 0.3, 0.7});
      auto cov3 = generate_full_covariance<Pose3>(to_sigmas6(the_axis, {0.1, 0.3, 0.7}));
      auto cov_trans = rotate_translate({90.}, {20., 0.}, cov2);
      auto cov3_trans = rotate_translate(to_rotation3(the_axis, 90.), to_translation3(the_axis, {20., 0.}), cov3);
      match_cov3_to_cov2(the_axis, cov2, cov3);
    }
  }

  static void pfp_covariance_all_axes_unit_test()
  {
    pfp_covariance_one_axis_unit_test(0);
    pfp_covariance_one_axis_unit_test(1);
    pfp_covariance_one_axis_unit_test(2);
  }

  static void pfp_covariance_pure_rotation_unit_test()
  {
    auto cov = generate_full_covariance<Pose3>({0.1, 0.2, 0.3, 0.5, 0.7, 1.1});

    // rotate 90 around x axis
    {
      auto cov_trans = rotate_only({90., 0., 0.}, cov);
      // interchange y (1 or 4) and z (2 or 5) axes for rotation and translation along diagonal
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), cov(2, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), cov(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), cov(5, 5), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), cov(4, 4), 1e-9);

      // Also interchange axes for off-diagonal elements. Note: The some axes map to
      // other axes in the negative direction so a positive correlation changes to a
      // negative correlation.
      EXPECT_DOUBLES_EQUAL(-cov_trans(2, 1), cov(2, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), cov(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(1, 0), cov(2, 0), 1e-9);

      EXPECT_DOUBLES_EQUAL(-cov_trans(5, 4), cov(5, 4), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 3), cov(4, 3), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(4, 3), cov(5, 3), 1e-9);

      // In the off-diagonal area that contains correlation between rotation state and
      // translation state, both rows and columns have to be interchanged.
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 1), cov(5, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 2), cov(4, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(5, 1), cov(4, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(4, 2), cov(5, 1), 1e-9);
    }

    // rotate 180 around x axis
    {
      auto cov_trans = rotate_only({180., 0., 0.}, cov);
      // no elements are changed on the diagonal
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), cov(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), cov(2, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), cov(4, 4), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), cov(5, 5), 1e-9);

      // Both the y and z axes are pointing in the negative direction.
      EXPECT_DOUBLES_EQUAL(-cov_trans(1, 0), cov(1, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(2, 0), cov(2, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(3, 0), cov(3, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(4, 0), cov(4, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(5, 0), cov(5, 0), 1e-9);
    }

    // rotate 90 around z axis and then 90 around y axis
    {
      auto cov_trans = rotate_only({0., 90., 90.}, cov);
      // x from y, y from z, z from x
      EXPECT_DOUBLES_EQUAL(cov_trans(0, 0), cov(1, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 1), cov(2, 2), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), cov(0, 0), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(3, 3), cov(4, 4), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), cov(5, 5), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), cov(3, 3), 1e-9);

      // Both the x and z axes are pointing in the negative direction.
      EXPECT_DOUBLES_EQUAL(-cov_trans(1, 0), cov(2, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), cov(0, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(3, 0), cov(4, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(-cov_trans(4, 0), cov(5, 1), 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 0), cov(3, 1), 1e-9);
    }
  }

  static void pfp_covariance_pure_translation_unit_test()
  {

    // translate along the x axis with variance in rotx
    {
      auto cov = generate_one_variable_covariance<Pose3>(0, 0.1);
      auto cov_trans = translate_only({20., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
    }

    // translate along the x axis with variance in x
    {
      auto cov = generate_one_variable_covariance<Pose3>(3, 0.1);
      auto cov_trans = translate_only({20., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 0., 1e-9);
    }

    // translate along the x axis with variance in roty
    {
      auto cov = generate_one_variable_covariance<Pose3>(1, 0.1);
      auto cov_trans = translate_only({20., 0., 0.}, cov);
      // The variance in roty causes an off-diagonal covariance and variance in z
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 1), 0.1 * 0.1 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 0.1 * 0.1 * 20. * 20., 1e-9);
    }

    // translate along the x axis with variance in roty and rotz
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

    // translate along the x axis with variance in y and z
    {
      auto cov = generate_two_variable_covariance<Pose3>(4, 5, 0.1, 0.3);
      auto cov_trans = translate_only({20., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 0.3 * 0.3, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 4), 0.1 * 0.3, 1e-9);
    }
  }

  static void pfp_covariance_rotation_translation_unit_test()
  {
    // rotate around x axis and translate along the x axis with variance in rotx
    {
      auto cov = generate_one_variable_covariance<Pose3>(0, 0.1);
      auto cov_trans = rotate_translate({90., 0., 0.}, {20., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
    }

    // rotate around x axis and translate along the x axis with variance in roty
    {
      auto cov = generate_one_variable_covariance<Pose3>(1, 0.1);
      auto cov_trans = rotate_translate({90., 0., 0.}, {20., 0., 0.}, cov);
      // interchange the y and z axes.
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 2), 0.1 * 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 2), -0.1 * 0.1 * 20., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0.1 * 0.1 * 20. * 20., 1e-9);
    }
  }

  static void pfp_covariance_pure_rotation_unit_test_xx()
  {
    auto cov = generate_full_covariance<Pose3>({0.1, 0.2, 0.3, 0.5, 0.7, 1.1});

    Pose3 t_world_body{Rot3::RzRyRx(180. * degree, 0. * degree, 0. * degree), Point3{0., 0., 0.}};


    std::cout << PoseWithCovariance::to_str(t_world_body) << std::endl;
    std::cout << PoseWithCovariance::to_str(cov) << std::endl;

    auto adj = t_world_body.AdjointMap();
    Matrix6 cov_trans = adj * cov * adj.transpose();

    std::cout << PoseWithCovariance::to_str(cov_trans) << std::endl;

  }

  static void pfp_covariance_transform_unit_test()
  {
    Vector6 sigma;
//    sigma << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1;
    sigma << 0., 0., 0.2, 0., 0., 0.1;
    Matrix6 cov;
    cov.setZero();
    cov.diagonal() = (sigma.array() * sigma.array()).matrix();

    auto set_off_diag = [&sigma, &cov](int r, int c) -> void
    {
      cov(r, c) = cov(c, r) = 1.0 * sigma(r) * sigma(c);
    };
//    set_off_diag(2, 5);

    Pose3 t_world_body{Rot3::RzRyRx(0. * degree, 0. * degree, 0. * degree), Point3{10., 0., 0.}};


    std::cout << PoseWithCovariance::to_str(t_world_body) << std::endl;
    std::cout << PoseWithCovariance::to_str(cov) << std::endl;

    auto adj = t_world_body.AdjointMap();
    Matrix6 cov_trans = adj * cov * adj.transpose();

    std::cout << PoseWithCovariance::to_str(cov_trans) << std::endl;
  }

  void pfp_pose_unit_test()
  {
    // Tests:
    // Pure rotation by 90 degrees
    //  interchange row/column
    //  some covariance is negative
    // Pure translation with one rotation sigma non-zero
    //  Introduces sigma in spatial and covariance
    // Pure translation with one spatial sigma non-zero
    //
//    pfp_covariance_pure_rotation_unit_test();
//    pfp_covariance_pure_translation_unit_test();
//    pfp_covariance_rotation_translation_unit_test();
//    pfp_covariance2_rotation_unit_test();
    pfp_covariance_all_axes_unit_test();
  }
}
