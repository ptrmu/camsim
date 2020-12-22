
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
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


  int sfm_isam_example()
  {
    // Define the camera calibration parameters
    gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));

    // Define the camera observation noise model, 1 pixel stddev
    auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points = createPoints();

    // Create the set of ground-truth poses
    std::vector<gtsam::Pose3> poses = createPoses();

    // Create an iSAM2 object. Unlike iSAM1, which performs periodic batch steps
    // to maintain proper linearization and efficient variable ordering, iSAM2
    // performs partial relinearization/reordering at each step. A parameter
    // structure is available that allows the user to set various properties, such
    // as the relinearization threshold and type of linear solver. For this
    // example, we we set the relinearization threshold small so the iSAM2 result
    // will approach the batch result.
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);

    // Create a Factor Graph and Values to hold the new data
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;

    // Loop over the poses, adding the observations to iSAM incrementally
    for (size_t i = 0; i < poses.size(); ++i) {
      // Add factors for each landmark observation
      for (size_t j = 0; j < points.size(); ++j) {
        gtsam::SimpleCamera camera(poses[i], *K);
        gtsam::Point2 measurement = camera.project(points[j]);
        graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2> >(
          measurement, measurementNoise, gtsam::Symbol('x', i), gtsam::Symbol('l', j), K);
      }

      // Add an initial guess for the current pose
      // Intentionally initialize the variables off from the ground truth
      static gtsam::Pose3 kDeltaPose(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                                     gtsam::Point3(0.05, -0.10, 0.20));
      initialEstimate.insert(gtsam::Symbol('x', i), poses[i] * kDeltaPose);

      // If this is the first iteration, add a prior on the first pose to set the
      // coordinate frame and a prior on the first landmark to set the scale Also,
      // as iSAM solves incrementally, we must wait until each is observed at
      // least twice before adding it to iSAM.
      if (i == 0) {
        // Add a prior on pose x0, 30cm std on x,y,z and 0.1 rad on roll,pitch,yaw
        static auto kPosePrior = gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << gtsam::Vector3::Constant(0.3), gtsam::Vector3::Constant(0.1))
            .finished());
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('x', 0), poses[0],
                                                                kPosePrior);

        // Add a prior on landmark l0
        static auto kPointPrior = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(gtsam::Symbol('l', 0), points[0],
                                                                 kPointPrior);

        // Add initial guesses to all observed landmarks
        // Intentionally initialize the variables off from the ground truth
        static gtsam::Point3 kDeltaPoint(-0.25, 0.20, 0.15);
        for (size_t j = 0; j < points.size(); ++j)
          initialEstimate.insert<gtsam::Point3>(gtsam::Symbol('l', j), points[j] + kDeltaPoint);

      } else {
        // Update iSAM with the new factors
        isam.update(graph, initialEstimate);
        // Each call to iSAM2 update(*) performs one iteration of the iterative
        // nonlinear solver. If accuracy is desired at the expense of time,
        // update(*) can be called additional times to perform multiple optimizer
        // iterations every step.
        isam.update();
        gtsam::Values currentEstimate = isam.calculateEstimate();
        std::cout << "****************************************************" << std::endl;
        std::cout << "Frame " << i << ": " << std::endl;
//        currentEstimate.print("Current estimate: ");

        // Clear the factor graph and values for the next iteration
        graph.resize(0);
        initialEstimate.clear();
      }
    }

    return EXIT_SUCCESS;
  }
}
