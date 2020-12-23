
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include "pfm_run.hpp"

namespace camsim
{
  void pfm_resection_projection(const gtsam::Cal3_S2 &camera_calibration,
                                const std::vector<gtsam::Point2> &corners_f_image,
                                const std::vector<gtsam::Point3> &corners_f_world,
                                const gtsam::Pose3 &camera_f_world_initial,
                                const gtsam::SharedNoiseModel &measurement_noise,
                                gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance)
  {
    // Create a factor graph and initial estimate
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(camera_calibration));

    auto camera_key = gtsam::Symbol('c', 0);

    // Add measurements to the factor graph
    for (size_t i = 0; i < corners_f_image.size(); ++i) {
      auto &corner_f_image = corners_f_image[i];
      auto point_key = gtsam::Symbol('l', i);
      graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
        corner_f_image,
        measurement_noise,
        camera_key,
        point_key,
        K);
    }

    // Add priors and initials on the corners to the factor graph
    auto pointConstrainedNoise = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_3x1);
    for (size_t i = 0; i < corners_f_world.size(); ++i) {
      auto &corner_f_world = corners_f_world[i];
      auto point_key = gtsam::Symbol('l', i);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(point_key, corner_f_world, pointConstrainedNoise);
      initial.insert(point_key, corner_f_world);
    }

    // Add initial estimate for camera pose
    initial.insert(camera_key, camera_f_world_initial);

    /* Optimize the graph and print results */
    auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    camera_f_world = result.at<gtsam::Pose3>(camera_key);

    gtsam::Marginals marginals(graph, result);
    camera_f_world_covariance = marginals.marginalCovariance(camera_key);
  }
}
