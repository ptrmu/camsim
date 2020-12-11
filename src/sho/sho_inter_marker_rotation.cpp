
#include "sho_run.hpp"

#include "fvlam/transform3_with_covariance.hpp"
#include "gtsam/geometry/Pose3.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace camsim
{

  /* Calculate the transform from one marker to another given
   * their transforms from a camera. The optimization is done
   * here so the error can be propagated. There is probably a
   * way to do this analytically but I don't know how.
   */
  fvlam::Transform3WithCovariance calc_t_marker0_marker1(
    fvlam::Transform3WithCovariance t_camera_marker0,
    fvlam::Transform3WithCovariance t_camera_marker1)
  {
    static const std::uint64_t camera_key = 0;
    static const std::uint64_t marker0_key = 1;
    static const std::uint64_t marker1_key = 2;

    // Prepare for an optimization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // The keys are used as
    // A prior on m0 has no uncertainty.
    auto m0_f_w_covariance = gtsam::noiseModel::Constrained::All(gtsam::Pose3::dimension);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      marker0_key, gtsam::Pose3(), m0_f_w_covariance);

    // Add the measurements
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      camera_key, marker0_key,
      t_camera_marker0.tf().to<gtsam::Pose3>(),
      gtsam::noiseModel::Gaussian::Covariance(t_camera_marker0.cov().matrix()));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      camera_key, marker1_key,
      t_camera_marker1.tf().to<gtsam::Pose3>(),
      gtsam::noiseModel::Gaussian::Covariance(t_camera_marker1.cov().matrix()));

    // Set the initial values
    auto t_marker0_camera_i = t_camera_marker0.tf().inverse().to<gtsam::Pose3>();
    auto t_camera_marker1_i = t_camera_marker1.tf().to<gtsam::Pose3>();
    initial.insert(marker0_key, gtsam::Pose3());
    initial.insert(camera_key, t_marker0_camera_i);
    initial.insert(marker1_key, t_marker0_camera_i * t_camera_marker1_i);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto m1_f_m0_calc = result.at<gtsam::Pose3>(marker1_key);
    gtsam::Pose3::Jacobian m1_f_m0_calc_covariance = marginals.marginalCovariance(marker1_key);

    return fvlam::Transform3WithCovariance(
      fvlam::Transform3::from(m1_f_m0_calc),
      fvlam::Transform3Covariance(m1_f_m0_calc_covariance));
  }

  int inter_marker_rotation()
  {
    // First set up the pose of the markers and camera.
    fvlam::Translate3 m0t{0, 0, 4};
    fvlam::Translate3 m1t{2, 3, 2};
    fvlam::Translate3 c0t{1, 0, 0};
    fvlam::Rotate3 m0r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 0);
    fvlam::Rotate3 m1r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 90 * M_PI / 180.);
    fvlam::Rotate3 c0r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 0);
    fvlam::Transform3 m0_f_w{m0r, m0t};
    fvlam::Transform3 m1_f_w{m1r, m1t};
    fvlam::Transform3 c0_f_w{c0r, c0t};

    auto cov_value = 0.1;
    fvlam::Transform3 t_c0_m0 = c0_f_w.inverse() * m0_f_w;
    fvlam::Transform3 t_c0_m1 = c0_f_w.inverse() * m1_f_w;
    fvlam::Transform3Covariance t_c0_m0_cov(fvlam::Transform3Covariance::Derived::Identity() * cov_value);
    fvlam::Transform3Covariance t_c0_m1_cov(fvlam::Transform3Covariance::Derived::Identity() * cov_value);
    fvlam::Transform3WithCovariance t_c0_m0_with_cov(t_c0_m0, t_c0_m0_cov);
    fvlam::Transform3WithCovariance t_c0_m1_with_cov(t_c0_m1, t_c0_m1_cov);

    std::cout << "t_c0_m0_with_cov    " << std::endl << t_c0_m0_with_cov.to_string() << std::endl;
    std::cout << "t_c0_m1_with_cov    " << std::endl << t_c0_m1_with_cov.to_string() << std::endl;

    auto t_m0_m1_with_cov = calc_t_marker0_marker1(t_c0_m0_with_cov, t_c0_m1_with_cov);

    std::cout << "t_m0_m1_with_cov    " << std::endl << t_m0_m1_with_cov.to_string() << std::endl;
    std::cout << "t_m0_m1_truth       " << std::endl << (t_c0_m0.inverse() * t_c0_m1).to_string() << std::endl;

    return 0;
  }
}