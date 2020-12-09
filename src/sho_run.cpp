
#include "sho_run.hpp"

#include <cstdlib>
#include <iostream>
#include <random>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>


namespace camsim
{
  const double degree = M_PI / 180;

  int shonan_rotation_averaging()
  {
    std::string inputFile("../src/data/pose3example.txt");
    int d, seed;

    // Seed random number generator
    static std::mt19937 rng(seed);

    gtsam::NonlinearFactorGraph::shared_ptr inputGraph;
    gtsam::Values::shared_ptr posesInFile;
    gtsam::Values poses;

    std::cout << "Running Shonan averaging for SO(3) on " << inputFile << std::endl;
    gtsam::ShonanAveraging3 shonan(inputFile);
    auto initial = shonan.initializeRandomly(rng);
    auto result = shonan.run(initial);

    result.first.print("result");

    // Parse file again to set up translation problem, adding a prior
    boost::tie(inputGraph, posesInFile) = gtsam::load3D(inputFile);
    auto priorModel = gtsam::noiseModel::Unit::Create(6);
    inputGraph->addPrior(0, posesInFile->at<gtsam::Pose3>(0), priorModel);
    inputGraph->print("inputGraph");

    std::cout << "recovering 3D translations" << std::endl;
    auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(*inputGraph);
    poses = gtsam::initialize::computePoses<gtsam::Pose3>(result.first, &poseGraph, false);

    posesInFile->print("posesInFile");
    poses.print("poses");

    return EXIT_SUCCESS;
  }


  template<class Pose>
  static gtsam::Values myComputePoses(const gtsam::Values &initialRot,
                                      gtsam::NonlinearFactorGraph *posegraph,
                                      bool singleIter = true)
  {
    const auto origin = Pose().translation();

    // Upgrade rotations to full poses
    gtsam::Values initialPose;
    for (const auto &key_value : initialRot) {
      gtsam::Key key = key_value.key;
      const auto &rot = initialRot.at<typename Pose::Rotation>(key);
      Pose initializedPose = Pose(rot, origin);
      initialPose.insert(key, initializedPose);
    }

    // Create optimizer
    gtsam::GaussNewtonParams params;
    if (singleIter) {
      params.maxIterations = 1;
    } else {
      params.setVerbosity("TERMINATION");
    }
    gtsam::GaussNewtonOptimizer optimizer(*posegraph, initialPose, params);
    gtsam::Values GNresult = optimizer.optimize();

    // put into Values structure
    gtsam::Values estimate;
    for (const auto &key_value : GNresult) {
      gtsam::Key key = key_value.key;
      const Pose &pose = GNresult.at<Pose>(key);
      estimate.insert(key, pose);
    }
    return estimate;
  }

  int shonan_RA_simple()
  {
    // This code uses shonan averaging to figure out the pose of three points..

    // First set up the pose of the markers and camera.
    gtsam::Point3 p_m0_f_w{-1, 0, 0};
    gtsam::Point3 p_m1_f_w{1, 0, 0};
    gtsam::Point3 p_c0_f_w{0, 1, 0};
    gtsam::Rot3 r_m0_f_w = gtsam::Rot3::RzRyRx(0, 0, 0);
    gtsam::Rot3 r_m1_f_w = gtsam::Rot3::RzRyRx(0, 0, 90 * degree);
    gtsam::Rot3 r_c0_f_w = gtsam::Rot3::RzRyRx(0, 0, -90.00 * degree);
    gtsam::Pose3 m0_f_w{r_m0_f_w, p_m0_f_w};
    gtsam::Pose3 m1_f_w{r_m1_f_w, p_m1_f_w};
    gtsam::Pose3 c0_f_w{r_c0_f_w, p_c0_f_w};

    auto cov_value = 0.1;
    auto sigma_value = 0.01;

    gtsam::Rot3 r_m0_f_c0 = r_c0_f_w.inverse() * r_m0_f_w;
    gtsam::Rot3 r_m1_f_c0 = r_c0_f_w.inverse() * r_m1_f_w;
    auto r_m0_f_c0_sigma = gtsam::Rot3::TangentVector::Constant(sigma_value);
    auto r_m1_f_c0_sigma = gtsam::Rot3::TangentVector::Constant(sigma_value);

    gtsam::Pose3 t_c0_m0 = c0_f_w.inverse() * m0_f_w;
    gtsam::Pose3 t_c0_m1 = c0_f_w.inverse() * m1_f_w;
    auto t_c0_m0_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);
    auto t_c0_m1_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);

    r_m0_f_c0.print("r_m0_f_c0");
    r_m1_f_c0.print("r_m1_f_c0");

    gtsam::ShonanAveraging3::Measurements measurements{};
    measurements.emplace_back(gtsam::BinaryMeasurement<gtsam::Rot3>{
      0, 1, r_m0_f_c0, gtsam::noiseModel::Diagonal::Sigmas(r_m0_f_c0_sigma)});
    measurements.emplace_back(gtsam::BinaryMeasurement<gtsam::Rot3>{
      0, 2, r_m1_f_c0, gtsam::noiseModel::Diagonal::Sigmas(r_m1_f_c0_sigma)});

    gtsam::ShonanAveraging3 shonan(measurements);
    auto shonan_initial = shonan.initializeRandomly();
    auto result = shonan.run(shonan_initial);

    std::cout << "error of Shonan Averaging " << result.second << std::endl;
    result.first.print("result");


    gtsam::NonlinearFactorGraph graph;

    // A prior on m0.
    auto priorModel = gtsam::noiseModel::Unit::Create(gtsam::Pose3::dimension);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(1, m0_f_w, priorModel);

    // Add the measurements
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 1, t_c0_m0, gtsam::noiseModel::Diagonal::Sigmas(t_c0_m0_sigma));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 2, t_c0_m1, gtsam::noiseModel::Diagonal::Sigmas(t_c0_m1_sigma));

//    auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(graph);
    auto poses = myComputePoses<gtsam::Pose3>(result.first, &graph, false);

    graph.print("graph ");
//    poseGraph.print("poseGraph ");
    poses.print("poses");

    return 0;
  }
}

