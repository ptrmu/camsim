
#include "pfp_run.hpp"

#include "gtsam/geometry/Pose2.h"
#include "gtsam/geometry/Pose3.h"
#include "pose_with_covariance.hpp"

using namespace gtsam;

namespace camsim
{
  const double degree = M_PI / 180;

  static void pfp_covariance_pure_rotation_unit_test()
  {
    Vector6 sigma;
    sigma << 0.1, 0.2, 0.3, 0.5, 0.7, 1.1;
    Matrix6 cov;
    cov.diagonal() = (sigma.array() * sigma.array()).matrix();
    for (int r = 0; r < 6; r += 1) {
      for (int c = r + 1; c < 6; c += 1) {
        cov(r, c) = cov(c, r) = 1.0 * sigma(r) * sigma(c);
      }
    }

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
    pfp_covariance_transform_unit_test();
  }
}
