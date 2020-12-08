
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

    // Parse file again to set up translation problem, adding a prior
    boost::tie(inputGraph, posesInFile) = gtsam::load3D(inputFile);
    auto priorModel = gtsam::noiseModel::Unit::Create(6);
    inputGraph->addPrior(0, posesInFile->at<gtsam::Pose3>(0), priorModel);

    std::cout << "recovering 3D translations" << std::endl;
    auto poseGraph = gtsam::initialize::buildPoseGraph<gtsam::Pose3>(*inputGraph);
    poses = gtsam::initialize::computePoses<gtsam::Pose3>(result.first, &poseGraph);

    return EXIT_SUCCESS;
  }
}

