
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
#include "../model.hpp"
#include "../pose_with_covariance.hpp"

namespace camsim
{
  const double degree = M_PI / 180;

  int pfp_simple()
  {
    // This sample explores how GTSAM rotates covariance matrices. Start with a pose and covariance. We are calling
    // it world. The we have a transform that we are calling t_world_body. We measure this rotation with zero error.
    // We put this original pose and transform measurement into the solver and find the pose body_f_world and the
    // covariance body_f_world_covariance. This new covariance is the original covariance rotated by t_world_body.

    gtsam::Symbol world_key('w', 0);
    gtsam::Symbol body_key('b', 0);

    // Create the world pose and a covariance in the world frame
    gtsam::Pose3 world{gtsam::Rot3::RzRyRx(0., 0., 0.),
                       gtsam::Point3{0., 0., 0.}};

    gtsam::Pose3::Jacobian world_covariance;
    world_covariance.setZero();
    world_covariance(0, 0) = 1.1e-2;
    world_covariance(1, 1) = 1.2e-2;
    world_covariance(2, 2) = 1.3e-2;
    world_covariance(3, 3) = 2.1e-2;
    world_covariance(4, 4) = 2.2e-2;
    world_covariance(5, 5) = 2.3e-2;
    int cor_0 = 1;
    int cor_1 = 2;
    double cor_c = 1.0;
    double cor_f = cor_c * world_covariance(cor_0, cor_0) * world_covariance(cor_1, cor_1);
    world_covariance(cor_0, cor_1) = cor_f;
    world_covariance(cor_1, cor_0) = cor_f;
    auto world_noise_model = gtsam::noiseModel::Gaussian::Covariance(world_covariance, false);

    // Create the body transform measurement and the noise model with no variance.
    double rx = 90 * degree;
    double ry = 90 * degree;
    double rz = 0 * degree;
    gtsam::Pose3 t_world_body{gtsam::Rot3::Ypr(rx, ry, rz), gtsam::Point3{0., 5., 0.}};
    auto t_world_body_measurement_noise = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);

    std::cout << "world" << std::endl << PoseWithCovariance::to_str(world) << std::endl;
    std::cout << "world_covariance" << std::endl << PoseWithCovariance::to_matrix_str(world_covariance, true) << std::endl;
    std::cout << "t_world_body" << std::endl << PoseWithCovariance::to_str(t_world_body) << std::endl;

    // Do the optimization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Build up the graph
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(world_key,
                                                            world, world_noise_model);
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(world_key,
                                                             body_key,
                                                             t_world_body,
                                                             t_world_body_measurement_noise);

    // Add the initial estimates
    initial.insert(world_key, world);
    initial.insert(body_key, t_world_body);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    // Get the body pose and the rotated covariance.
    gtsam::Marginals marginals(graph, result);
    auto body_f_world = result.at<gtsam::Pose3>(body_key);
    gtsam::Pose3::Jacobian body_f_world_covariance = marginals.marginalCovariance(body_key);

    std::cout << "body_f_world" << std::endl << PoseWithCovariance::to_str(body_f_world) << std::endl;
    std::cout << "body_f_world_covariance" << std::endl << PoseWithCovariance::to_matrix_str(body_f_world_covariance, true)
              << std::endl;

    std::cout << std::endl;

    // Lets see where this covariance comes from.
    // First try rotating the rotated covariance back to the body frame. Just work with the rotation part.
    auto body_f_world_rotation = body_f_world.rotation();
    auto body_f_world_rot_covariance = body_f_world_covariance.block<3, 3>(0, 0);

    std::cout << "Transform rotation part of body covariance back to world frame. Should match original." << std::endl;
    gtsam::Rot3::Jacobian r_covariance = body_f_world_rotation.matrix() *
                                         body_f_world_rot_covariance * body_f_world_rotation.matrix().transpose();
    std::cout << "rotation part of body_f_world_covariance in world frame" << std::endl <<
              PoseWithCovariance::to_matrix_str(r_covariance, true) << std::endl;

    std::cout << "Transform full body covariance back to world frame using AdjointMap. Should match original."
              << std::endl;
    gtsam::Pose3::Jacobian adjoint_map = body_f_world.AdjointMap();
    gtsam::Pose3::Jacobian r_adj_cov = adjoint_map * body_f_world_covariance * adjoint_map.transpose();
    std::cout << "body_f_world_covariance in world frame" << std::endl <<
              PoseWithCovariance::to_matrix_str(r_adj_cov, true) << std::endl;

    return 0;
  }

  int pfp_simplest()
  {
    // Create the world and body poses in the world frame
    gtsam::Pose3 t_world_world{};
    gtsam::Pose3 t_world_body{gtsam::Rot3::RzRyRx(90. * degree, 0. * degree, 0. * degree), gtsam::Point3::Zero()};

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Add factors to the graph
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      gtsam::Symbol('w', 0),
      t_world_world,
      gtsam::noiseModel::Diagonal::Variances(
        (gtsam::Vector(6) << 0.11, 0.12, 0.13, 0.21, 0.22, 0.23).finished()));

    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('w', 0),
      gtsam::Symbol('b', 0),
      t_world_body,
      gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1));

    // Add the initial estimates
    initial.insert(gtsam::Symbol('w', 0), t_world_world);
    initial.insert(gtsam::Symbol('b', 0), t_world_body);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto result_body_f_world = result.at<gtsam::Pose3>(gtsam::Symbol('b', 0));
    gtsam::Matrix6 world_f_world_covariance = marginals.marginalCovariance(gtsam::Symbol('w', 0));
    gtsam::Matrix6 body_f_world_covariance = marginals.marginalCovariance(gtsam::Symbol('b', 0));

    std::cout << "Example of how covariance is transformed by a Pose3 transformation." << std::endl;
    std::cout << "original world_f_world_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(world_f_world_covariance, true) << std::endl;
    std::cout << "result_body_f_world" << std::endl << PoseWithCovariance::to_str(result_body_f_world) << std::endl;
    std::cout << "body_f_world_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(body_f_world_covariance, true) << std::endl;

    return 0;
  }

  int pfp_duplicate_prior()
  {
    // Create the world and body poses in the world frame
    gtsam::Point3 p0{0., 0., 0.};
    auto p0_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{0.1, 0.1, 0.1});
    gtsam::Point3 p1{10., 10., 10.};
    auto p1_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3{10., 1., 0.1});
    gtsam::Pose3 t_world_body{gtsam::Rot3::RzRyRx(90. * degree, 0. * degree, 0. * degree), gtsam::Point3::Zero()};

    std::cout << "Example of how multiple priors are combined." << std::endl;
    std::cout << "p0" << std::endl << PoseWithCovariance::to_row_str(p0.transpose()) << std::endl;
    std::cout << "p0_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(gtsam::Matrix3{p0_noise->covariance()}, true) << std::endl;
    std::cout << "p1" << std::endl << PoseWithCovariance::to_row_str(p1.transpose()) << std::endl;
    std::cout << "p1_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(gtsam::Matrix3{p1_noise->covariance()}, true) << std::endl;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // Add factors to the graph
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
      gtsam::Symbol('w', 0),
      p0, p0_noise);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
      gtsam::Symbol('w', 0),
      p1, p1_noise);

    // Add the initial estimates
    initial.insert(gtsam::Symbol('w', 0), p1);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto result_p0 = result.at<gtsam::Point3>(gtsam::Symbol('w', 0));
    gtsam::Matrix3 result_p0_covariance = marginals.marginalCovariance(gtsam::Symbol('w', 0));

    std::cout << "result_p0" << std::endl << PoseWithCovariance::to_row_str(result_p0.transpose()) << std::endl;
    std::cout << "result_p0_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(result_p0_covariance, true) << std::endl;

    // The resulting position coordinates are averaged based on their uncertainty. But notice the
    // counter intuitive result that even though the two points had radically different values, the
    // resulting uncertainty is small.
    return 0;
  }

  int pfp_marker_marker_transform()
  {
    // This code calculates the pose transformation between two markers given
    // the measurement of their transforms from a common camera.

    // First set up the pose of the markers and camera.
    gtsam::Point3 m0t{0, 0, 0};
    gtsam::Point3 m1t{2, 0, 0};
    gtsam::Point3 c0t{1, 0, 0};
    gtsam::Rot3 m0r = gtsam::Rot3::RzRyRx(0, 0, 0);
    gtsam::Rot3 m1r = gtsam::Rot3::RzRyRx(0, 0, 90 * degree);
    gtsam::Rot3 c0r = gtsam::Rot3::RzRyRx(0, 0, 0);
    gtsam::Pose3 m0_f_w{m0r, m0t};
    gtsam::Pose3 m1_f_w{m1r, m1t};
    gtsam::Pose3 c0_f_w{c0r, c0t};

    auto cov_value = 0.1;
    auto sigma_value = std::sqrt(cov_value);
    gtsam::Pose3 t_c0_m0 = c0_f_w.inverse() * m0_f_w;
    gtsam::Pose3 t_c0_m1 = c0_f_w.inverse() * m1_f_w;
    auto t_c0_m0_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);
    auto t_c0_m1_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);


    // Prepare for an optimization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // A prior on m0 has no uncertainty.
    auto m0_f_w_covariance = gtsam::noiseModel::Constrained::All(gtsam::Pose3::dimension);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      gtsam::Symbol('m', 0), m0_f_w, m0_f_w_covariance);

    // Add the measurements
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('c', 0), gtsam::Symbol('m', 0),
      t_c0_m0, gtsam::noiseModel::Diagonal::Sigmas(t_c0_m0_sigma));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      gtsam::Symbol('c', 0), gtsam::Symbol('m', 1),
      t_c0_m1, gtsam::noiseModel::Diagonal::Sigmas(t_c0_m1_sigma));

    // Set the initial values
    initial.insert(gtsam::Symbol('m', 0), m0_f_w);
    initial.insert(gtsam::Symbol('m', 1), m1_f_w);
    initial.insert(gtsam::Symbol('c', 0), c0_f_w);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto m1_f_m0_calc = result.at<gtsam::Pose3>(gtsam::Symbol('m', 1));
    gtsam::Pose3::Jacobian m1_f_m0_calc_covariance = marginals.marginalCovariance(gtsam::Symbol('m', 1));

    std::cout << "m1_f_w_calc" << std::endl << PoseWithCovariance::to_str(m1_f_m0_calc) << std::endl;
    std::cout << "m1_f_w_calc_covariance" << std::endl
              << PoseWithCovariance::to_matrix_str(m1_f_m0_calc_covariance, true) << std::endl;

    return 0;
  }
}
