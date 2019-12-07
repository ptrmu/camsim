
#include "pfp_run.hpp"

#include "gtsam/geometry/Point3.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include "model.hpp"
#include "pose_with_covariance.hpp"

namespace camsim
{
  const double degree = M_PI / 180;

  static void pfp_simple()
  {
    // Create the world pose
    gtsam::Pose3 world{gtsam::Rot3::RzRyRx(0., 0., 0.),
                       gtsam::Point3{0., 0., 0.}};

    gtsam::Matrix6 cov;
    cov.setZero();
    cov(0, 0) = 1.1e-2;
    cov(1, 1) = 1.2e-2;
    cov(2, 2) = 1.3e-2;
    cov(3, 3) = 2.1e-2;
    cov(4, 4) = 2.2e-2;
    cov(5, 5) = 2.3e-2;
    int cor_0 = 3;
    int cor_1 = 5;
    double cor_c = 1.0;
    double cor_f = cor_c * cov(cor_0, cor_0) * cov(cor_1, cor_1);
    cov(cor_0, cor_1) = cor_f;
    cov(cor_1, cor_0) = cor_f;
    auto world_noise_model = gtsam::noiseModel::Gaussian::Covariance(cov, false);

    gtsam::Symbol world_key('w', 0);


    // Create the body pose
    gtsam::Pose3 body_f_world{gtsam::Rot3::Ypr(90. * degree, 90. * degree, 0. * degree),
                              gtsam::Point3{0., 0., 0.}};

    // Create the measurement constraint
    auto body_f_world_measurement_noise = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);

    gtsam::Symbol body_key('b', 0);


    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Build up the graph
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(world_key,
                                                            world, world_noise_model);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(world_key,
                                                             body_key,
                                                             body_f_world,
                                                             body_f_world_measurement_noise);

    // Add the initial estimates
    initial.insert(world_key, world);
    initial.insert(body_key, body_f_world);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto result_body_f_world = result.at<gtsam::Pose3>(body_key);
    gtsam::Matrix6 result_body_f_world_covariance = marginals.marginalCovariance(body_key);

    std::cout << PoseWithCovariance::to_str(result_body_f_world) << std::endl;
    std::cout << PoseWithCovariance::to_str(result_body_f_world_covariance) << std::endl;

    gtsam::Matrix3 t_cov{};
    for (int r = 0; r < 3; r += 1) {
      for (int c = 0; c < 3; c += 1) {
        t_cov(r, c) = result_body_f_world_covariance(r + 3, c + 3);
      }
    }
    std::cout << t_cov << std::endl;
    gtsam::Matrix3 r_t_cov =
      result_body_f_world.rotation().matrix() * t_cov * result_body_f_world.rotation().matrix().transpose();
    std::cout << r_t_cov << std::endl;

    gtsam::Matrix6 rot_cov{};
    rot_cov.setZero();
    gtsam::Matrix3 rot = result_body_f_world.rotation().matrix();
    for (int r = 0; r < 3; r += 1) {
      for (int c = 0; c < 3; c += 1) {
        rot_cov(r, c) = rot(r, c);
        rot_cov(r + 3, c + 3) = rot(r, c);
      }
    }

    gtsam::Matrix6 r_cov = rot_cov * result_body_f_world_covariance * rot_cov.transpose();
    std::cout << r_cov << std::endl;
  }

  using namespace gtsam;

  static void pfp_simplest()
  {
    // Create the world and body poses in the world frame
    Pose3 t_world_world{};
    Pose3 t_world_body{Rot3::RzRyRx(90. * degree, 0. * degree, 0. * degree), Point3{}};

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Add factors to the graph
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      gtsam::Symbol('w', 0),
      t_world_world,
      noiseModel::Diagonal::Variances((Vector(6) << 0.11, 0.12, 0.13, 0.21, 0.22, 0.23).finished()));

    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('w', 0),
      gtsam::Symbol('b', 0),
      t_world_body,
      noiseModel::Constrained::MixedSigmas(Z_6x1));

    // Add the initial estimates
    initial.insert(gtsam::Symbol('w', 0), t_world_world);
    initial.insert(gtsam::Symbol('b', 0), t_world_body);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto result_body_f_world = result.at<gtsam::Pose3>(gtsam::Symbol('b', 0));
    gtsam::Matrix6 result_body_f_world_covariance = marginals.marginalCovariance(gtsam::Symbol('b', 0));

    std::cout << result_body_f_world << std::endl;
    std::cout << result_body_f_world_covariance << std::endl;
  }
}

int main()
{
//  camsim::pfp_simple();
//  camsim::odometry_example_3d();
  camsim::pfp_pose_unit_test();
  return EXIT_SUCCESS;
}
