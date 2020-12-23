
#include "sho_run.hpp"

#include <cstdlib>
#include <iostream>
#include <random>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/InitializePose.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>

#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"

namespace camsim
{
  const double degree = M_PI / 180;

  int shonan_rotation_averaging()
  {
    std::string inputFile("../src/data/pose3example.txt");
    int seed;

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
    fvlam::Translate3 c0_f_w_translate{0, 1, 0};
    fvlam::Translate3 m0_f_w_translate{-1, 0, 0};
    fvlam::Translate3 m1_f_w_translate{1, 0, 0};
    fvlam::Rotate3 r_c0_f_w = fvlam::Rotate3::RzRyRx(0, 0, -90.00 * degree);
    fvlam::Rotate3 r_m0_f_w = fvlam::Rotate3::RzRyRx(0, 0, 0);
    fvlam::Rotate3 r_m1_f_w = fvlam::Rotate3::RzRyRx(45 * degree, -45 * degree, 45 * degree);
    fvlam::Transform3 c0_f_w{r_c0_f_w, c0_f_w_translate};
    fvlam::Transform3 m0_f_w{r_m0_f_w, m0_f_w_translate};
    fvlam::Transform3 m1_f_w{r_m1_f_w, m1_f_w_translate};

    auto sigma_value = 0.1;

    fvlam::Rotate3 r_m0_f_c0 = r_c0_f_w.inverse() * r_m0_f_w;
    fvlam::Rotate3 r_m1_f_c0 = r_c0_f_w.inverse() * r_m1_f_w;
    auto r_m0_f_c0_sigma = gtsam::Rot3::TangentVector::Constant(sigma_value);
    auto r_m1_f_c0_sigma = gtsam::Rot3::TangentVector::Constant(sigma_value);

    fvlam::Transform3 t_c0_m0 = c0_f_w.inverse() * m0_f_w;
    fvlam::Transform3 t_c0_m1 = c0_f_w.inverse() * m1_f_w;
    auto t_c0_m0_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);
    auto t_c0_m1_sigma = gtsam::Pose3::TangentVector::Constant(sigma_value);

    std::cout << "c0_f_w  " << c0_f_w.to_string() << std::endl;
    std::cout << "m0_f_w  " << m0_f_w.to_string() << std::endl;
    std::cout << "m1_f_w  " << m1_f_w.to_string() << std::endl;
    std::cout << "t_c0_m0 " << t_c0_m0.to_string() << std::endl;
    std::cout << "t_c0_m1 " << t_c0_m1.to_string() << std::endl;

    gtsam::ShonanAveraging3::Measurements measurements{};
    measurements.emplace_back(gtsam::BinaryMeasurement<gtsam::Rot3>{0, 1,
                                                                    r_m0_f_c0.to<gtsam::Rot3>(),
                                                                    gtsam::noiseModel::Diagonal::Sigmas(
                                                                      r_m0_f_c0_sigma)});
    measurements.emplace_back(gtsam::BinaryMeasurement<gtsam::Rot3>{0, 2,
                                                                    r_m1_f_c0.to<gtsam::Rot3>(),
                                                                    gtsam::noiseModel::Diagonal::Sigmas(
                                                                      r_m1_f_c0_sigma)});

    gtsam::ShonanAveraging3 shonan(measurements);
    auto shonan_initial = shonan.initializeRandomly();
    auto result = shonan.run(shonan_initial);
    std::cout << "error of Shonan Averaging " << result.second << std::endl;

    std::cout << "r_c0_f_w_calc  " << fvlam::Rotate3::from(result.first.at<gtsam::Rot3>(0)).to_string() << std::endl;
    std::cout << "r_m0_f_w_calc  " << fvlam::Rotate3::from(result.first.at<gtsam::Rot3>(1)).to_string() << std::endl;
    std::cout << "r_m1_f_w_calc  " << fvlam::Rotate3::from(result.first.at<gtsam::Rot3>(2)).to_string() << std::endl;

    // Prepare to find the translations.
    gtsam::NonlinearFactorGraph graph;

    // A prior on m0.
    auto priorModel = gtsam::noiseModel::Unit::Create(gtsam::Pose3::dimension);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(1, m0_f_w.to<gtsam::Pose3>(), priorModel);

    // Add the measurements
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 1, t_c0_m0.to<gtsam::Pose3>(), gtsam::noiseModel::Diagonal::Sigmas(t_c0_m0_sigma));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      0, 2, t_c0_m1.to<gtsam::Pose3>(), gtsam::noiseModel::Diagonal::Sigmas(t_c0_m1_sigma));

    auto poses = myComputePoses<gtsam::Pose3>(result.first, &graph, false);

    auto c0_f_w_calc = fvlam::Transform3::from(poses.at<gtsam::Pose3>(0));
    auto m0_f_w_calc = fvlam::Transform3::from(poses.at<gtsam::Pose3>(1));
    auto m1_f_w_calc = fvlam::Transform3::from(poses.at<gtsam::Pose3>(2));

    std::cout << "c0_f_w_calc  " << c0_f_w_calc.to_string() << std::endl;
    std::cout << "m0_f_w_calc  " << m0_f_w_calc.to_string() << std::endl;
    std::cout << "m1_f_w_calc  " << m1_f_w_calc.to_string() << std::endl;

    return 0;
  }

  int test_rotate3()
  {
    int ret = 0;
    std::vector<double> test_angles = {-89.999, -60, -45, -30, 0, 30, 45, 60, 89.999};
    for (auto &a : test_angles)
      a *= degree;
    for (auto x : test_angles)
      for (auto y : test_angles)
        for (auto z : test_angles) {
          auto v = fvlam::Rotate3::RzRyRx(x, y, z).xyz();
          if (!gtsam::fpEqual(v(0), x, 1.0e-6) ||
              !gtsam::fpEqual(v(1), y, 1.0e-6) ||
              !gtsam::fpEqual(v(2), z, 1.0e-6)) {
            std::cout << "Angle error "
                      << v(0) / degree << "(" << x / degree << ") "
                      << v(1) / degree << "(" << y / degree << ") "
                      << v(2) / degree << "(" << z / degree << ") "
                      << std::endl;
            ret = 1;
          }
          v = gtsam::Rot3(fvlam::Rotate3::RzRyRx(x, y, z).rotation_matrix()).xyz();
          if (!gtsam::fpEqual(v(0), x, 1.0e-6) ||
              !gtsam::fpEqual(v(1), y, 1.0e-6) ||
              !gtsam::fpEqual(v(2), z, 1.0e-6)) {
            std::cout << "GTSAM difference error "
                      << v(0) / degree << "(" << x / degree << ") "
                      << v(1) / degree << "(" << y / degree << ") "
                      << v(2) / degree << "(" << z / degree << ") "
                      << std::endl;
            ret = 1;
          }
        }
    return ret;
  }
}

