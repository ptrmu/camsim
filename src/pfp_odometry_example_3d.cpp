
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

using namespace std;
using namespace gtsam;

namespace camsim
{
  int odometry_example_3d()
  {
    // Create an empty nonlinear factor graph
    NonlinearFactorGraph graph;

    // Add a prior on the first pose, setting it to the origin
    // A prior factor consists of a mean and a noise model (covariance matrix)
    Rot3 identity = Rot3::Ypr(0.0, 0.0, 0.0);
    Pose3 priorMean(identity, Point3{}); // prior at origin
    Vector6 priorSigmas;
    priorSigmas << 0.001, 0.001, 0.1, 0.3, 0.3, 0.0001;
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);
    graph.emplace_shared<PriorFactor<Pose3> >(1, priorMean, priorNoise);

    // Add odometry factors
    Point3 odometry_t(2.0, 0.0, 0.0);
    Pose3 odometry(identity, odometry_t);
    // For simplicity, we will use the same noise model for each odometry factor
    Vector6 odometrySigmas;
    odometrySigmas << 0.001, 0.001, 0.1, 0.2, 0.2, 0.0001;
    noiseModel::Diagonal::shared_ptr odometryNoise =  noiseModel::Diagonal::Sigmas(odometrySigmas);
    // Create odometry (Between) factors between consecutive poses
    graph.emplace_shared<BetweenFactor<Pose3> >(1, 2, odometry, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose3> >(2, 3, odometry, odometryNoise);
    graph.print("\nFactor Graph:\n"); // print


    // Create the data structure to hold the initialEstimate estimate to the solution
    // For illustrative purposes, these have been deliberately set to incorrect values
    Values initial;
    initial.insert(1, Pose3(Rot3::Ypr(0.2  , 0.1 , -0.2) , Point3(0.5, 0.0, 0.1)));
    initial.insert(2, Pose3(Rot3::Ypr(-0.2 , -0.1, -0.1) , Point3(2.3, 0.1, 0.1)));
    initial.insert(3, Pose3(Rot3::Ypr(0.1  , 0.1 , 0.1 ) , Point3(4.1, 0.1, 0.1)));
    initial.print("\nInitial Estimate:\n"); // print

    // optimize using Levenberg-Marquardt optimization
    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("Final Result:\n");

    // Calculate and print marginal covariances for all variables
    cout.precision(2);
    Marginals marginals(graph, result);
    cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
    cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
    cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;

    return 0;
  }
}
