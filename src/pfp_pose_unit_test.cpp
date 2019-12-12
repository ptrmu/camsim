
#include "pfp_run.hpp"

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "pose_with_covariance.hpp"

using namespace gtsam;

namespace camsim
{
  const double degree = M_PI / 180;

  template<class TPose>
  static typename TPose::Jacobian generate_full_covariance()
  {
    typename TPose::TangentVector sigma{};
    if (TPose::dimension == 3) {
      sigma << 0.1, 0.2, 0.3;
    } else {
      sigma << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1;
    }
    typename TPose::Jacobian cov{};
    cov.diagonal() = (sigma.array() * sigma.array()).matrix();
    for (int r = 1; r < TPose::dimension; r += 1) {
      for (int c = r - 1; c >= 0; c -= 1) {
        cov(r, c) = cov(c, r) = 1.0 * sigma(r) * sigma(c);
      }
    }
    return cov;
  }

  static void pfp_2d_covariance_rotation_unit_test()
  {
//    auto sigma = generate_full_covariance<Pose2>();
  }

#define EXPECT_DOUBLES_EQUAL(expected, actual, threshold)\
{ double actualTemp = actual; \
  double expectedTemp = expected; \
  if (!std::isfinite(actualTemp) || !std::isfinite(expectedTemp) || std::fabs ((expectedTemp)-(actualTemp)) > threshold) \
    { std::cout << "Failure file:" << __FILE__ << " line:" << __LINE__ << std::endl; } }

  static Pose3::Jacobian generate_full_covariance()
  {
    auto sigma = (Pose3::TangentVector{} << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1).finished();
    Pose3::Jacobian cov{};
    cov.diagonal() = (sigma.array() * sigma.array()).matrix();
    for (int r = 1; r < Pose3::dimension; r += 1) {
      for (int c = r - 1; c >= 0; c -= 1) {
        cov(r, c) = cov(c, r) = 1.0 * sigma(r) * sigma(c);
      }
    }
    return cov;
  }

  static Pose3::Jacobian generate_one_variable_covariance(int idx)
  {
    double sigma = 0.1;
    Pose3::Jacobian cov{};
    cov.setZero();
    cov(idx, idx) = sigma * sigma;
    return cov;
  }

  static Pose3::Jacobian generate_two_variable_covariance(int idx0, int idx1)
  {
    double sigma0 = 0.1;
    double sigma1 = 0.3;
    Pose3::Jacobian cov{};
    cov.setZero();
    cov(idx0, idx0) = sigma0 * sigma0;
    cov(idx1, idx1) = sigma1 * sigma1;
    cov(idx0, idx1) = cov(idx1, idx0) = sigma0 * sigma1;
    return cov;
  }

  static Pose3::Jacobian transform_only(
    const Pose3 &wTb,
    const Pose3::Jacobian &cov)
  {
    // transform the covariance with the AdjointMap
    auto adjointMap = wTb.AdjointMap();
    Matrix6 cov_trans = adjointMap * cov * adjointMap.transpose();

    std::cout << PoseWithCovariance::to_str(wTb) << std::endl;
    std::cout << PoseWithCovariance::to_str(cov) << std::endl;
    std::cout << PoseWithCovariance::to_str(cov_trans) << std::endl << std::endl;

    return cov_trans;
  }

  static Pose3::Jacobian rotate_only(
    const std::array<double, Pose3::dimension> r,
    const Pose3::Jacobian &cov)
  {
    // Create a rotation only transform
    Pose3 wTb{Rot3::RzRyRx(r[0] * degree, r[1] * degree, r[2] * degree), Point3{}};

    return transform_only(wTb, cov);
  }

  static Pose3::Jacobian translate_only(
    const std::array<double, Pose3::dimension> t,
    const Pose3::Jacobian &cov)
  {
    // Create a rotation only transform
    Pose3 wTb{Rot3{}, Point3{t[0], t[1], t[2]}};

    return transform_only(wTb, cov);
  }

  static void pfp_covariance_pure_rotation_unit_test()
  {
    auto cov = generate_full_covariance();

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
      auto cov = generate_one_variable_covariance(0);
      auto cov_trans = translate_only({10., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(1, 0), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(2, 0), 0., 1e-9);
    }

    // translate along the x axis with variance in x
    {
      auto cov = generate_one_variable_covariance(3);
      auto cov_trans = translate_only({10., 0., 0.}, cov);
      EXPECT_DOUBLES_EQUAL(cov_trans(4, 4), 0., 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 0., 1e-9);
    }

    // translate along the x axis with variance in roty
    {
      auto cov = generate_one_variable_covariance(1);
      auto cov_trans = translate_only({10., 0., 0.}, cov);
      // The variance in roty causes an off-diagonal covariance and variance in z
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 1), 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 10. * 0.1, 1e-9);
    }

    // translate along the x axis with variance in roty and rotz
    {
      auto cov = generate_two_variable_covariance(1, 2);
      auto cov_trans = translate_only({10., 0., 0.}, cov);
      // The variance in roty causes an off-diagonal covariance and variance in z
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 1), 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 10. * 0.1, 1e-9);
    }

    // translate along the x axis with variance in y and z
    {
      auto cov = generate_two_variable_covariance(4, 5);
      auto cov_trans = translate_only({10., 0., 0.}, cov);
      // The variance in roty causes an off-diagonal covariance and variance in z
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 1), 0.1, 1e-9);
      EXPECT_DOUBLES_EQUAL(cov_trans(5, 5), 10. * 0.1, 1e-9);
    }

  }

  static void pfp_covariance_pure_rotation_unit_test_xx()
  {
    auto cov = generate_full_covariance<Pose3>();

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
    pfp_covariance_pure_translation_unit_test();
  }
}
