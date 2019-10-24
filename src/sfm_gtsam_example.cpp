
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include "sfm_run.hpp"

namespace camsim
{
  static std::vector<gtsam::Point3> createPoints()
  {

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.emplace_back(gtsam::Point3(10.0, 10.0, 10.0));
    points.emplace_back(gtsam::Point3(-10.0, 10.0, 10.0));
    points.emplace_back(gtsam::Point3(-10.0, -10.0, 10.0));
    points.emplace_back(gtsam::Point3(10.0, -10.0, 10.0));
    points.emplace_back(gtsam::Point3(10.0, 10.0, -10.0));
    points.emplace_back(gtsam::Point3(-10.0, 10.0, -10.0));
    points.emplace_back(gtsam::Point3(-10.0, -10.0, -10.0));
    points.emplace_back(gtsam::Point3(10.0, -10.0, -10.0));

    return points;
  }

  static std::vector<gtsam::Pose3> createPoses(
    const gtsam::Pose3 &init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0)),
    const gtsam::Pose3 &delta = gtsam::Pose3(gtsam::Rot3::Ypr(0, -M_PI / 4, 0),
                                             gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
    int steps = 8)
  {

    // Create the set of ground-truth poses
    // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
    std::vector<gtsam::Pose3> poses;
    int i = 1;
    poses.push_back(init);
    for (; i < steps; ++i) {
      poses.push_back(poses[i - 1].compose(delta));
    }

    return poses;
  }

  int sfm_gtsam_example()
  {
    // Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // Define the camera observation noise model
    auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

    // Create the set of ground-truth landmarks
    auto points = createPoints();

    // Create the set of ground-truth poses
    auto poses = createPoses();

    // Create a factor graph
    gtsam::NonlinearFactorGraph graph;

    // Add a prior on pose x1. This indirectly specifies where the origin is.
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise =
      gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.3),
          gtsam::Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', 0), poses[0],
                                                            poseNoise); // add directly to graph

    // Simulated measurements from each camera pose, adding them to the factor graph
    for (size_t i = 0; i < poses.size(); ++i) {
      gtsam::SimpleCamera camera(poses[i], *K);
      for (size_t j = 0; j < points.size(); ++j) {
        gtsam::Point2 measurement = camera.project(points[j]);
        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >
          (measurement, measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K);
      }
    }

    // Because the structure-from-motion problem has a scale ambiguity, the problem is still under-constrained
    // Here we add a prior on the position of the first landmark. This fixes the scale by indicating the distance
    // between the first camera and the first landmark. All other landmark positions are interpreted using this scale.
    gtsam::noiseModel::Isotropic::shared_ptr pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(gtsam::Symbol('l', 0), points[0],
                                                             pointNoise); // add directly to graph
    graph.print("Factor Graph:\n");

    // Create the data structure to hold the initial estimate to the solution
    // Intentionally initialize the variables off from the ground truth
    gtsam::Values initialEstimate;
    for (size_t i = 0; i < poses.size(); ++i)
      initialEstimate.insert(gtsam::Symbol('x', i),
                             poses[i].compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                                           gtsam::Point3(0.05, -0.10, 0.20))));
    for (size_t j = 0; j < points.size(); ++j)
      initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', j), points[j] + gtsam::Point3(-0.25, 0.20, 0.15));
    initialEstimate.print("Initial Estimates:\n");

    /* Optimize the graph and print results */
    gtsam::Values result = gtsam::DoglegOptimizer(graph, initialEstimate).optimize();
    result.print("Final results:\n");
    std::cout << "initial error = " << graph.error(initialEstimate) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;

    return EXIT_SUCCESS;
  }
}