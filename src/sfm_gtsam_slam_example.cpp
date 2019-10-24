
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>

#include "sfm_run.hpp"

namespace camsim
{
  int sfm_gtsam_slam_example()
  {

    // Read graph from file
    std::string g2oFile{"../src/data/pose3example.txt"};

    gtsam::NonlinearFactorGraph::shared_ptr graph;
    gtsam::Values::shared_ptr initial;
    bool is3D = true;
    boost::tie(graph, initial) = gtsam::readG2o(g2oFile, is3D);

    // Add prior on the first key
    gtsam::NonlinearFactorGraph graphWithPrior = *graph;
    gtsam::noiseModel::Diagonal::shared_ptr priorModel = //
      gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
    gtsam::Key firstKey = 0;
    for (const gtsam::Values::ConstKeyValuePair &key_value: *initial) {
      std::cout << "Adding prior to g2o file " << std::endl;
      firstKey = key_value.key;
      graphWithPrior.add(gtsam::PriorFactor<gtsam::Pose3>(firstKey, gtsam::Pose3(), priorModel));
      break;
    }

    std::cout << "Optimizing the factor graph" << std::endl;
    gtsam::GaussNewtonParams params;
    params.setVerbosity("TERMINATION"); // this will show info about stopping conditions
    gtsam::GaussNewtonOptimizer optimizer(graphWithPrior, *initial, params);
    gtsam::Values result = optimizer.optimize();
    std::cout << "Optimization complete" << std::endl;

    std::cout << "initial error=" << graph->error(*initial) << std::endl;
    std::cout << "final error=" << graph->error(result) << std::endl;

    result.print("result");

    return 0;
  }
}
