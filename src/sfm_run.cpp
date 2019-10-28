
#include "sfm_run.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

#include "sfm_model.hpp"

namespace camsim
{
  static void simple_sfm(const SfmModel &sfm_model)
  {
    gtsam::NonlinearFactorGraph graph;

    // Add a prior factor on the first marker.
    // 100cm std on x,y,z 0.3 rad on roll,pitch,yaw
    auto marker_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1).finished());
    auto prior_maker_f_world = sfm_model.markers_.pose_f_worlds_[0];
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('m', 0),
                                                            sfm_model.markers_.pose_f_worlds_[0],
                                                            marker_noise);

    // Add the between factors
    for (size_t icam = 0; icam < sfm_model.cameras_.pose_f_worlds_.size(); icam += 1) {
      for (size_t imar = 0; imar < sfm_model.markers_.pose_f_worlds_.size(); imar += 1) {
        auto &camera_f_world = sfm_model.cameras_.pose_f_worlds_[icam];
        auto &marker_f_world = sfm_model.markers_.pose_f_worlds_[imar];
        gtsam::Pose3 marker_f_camera = camera_f_world.inverse() * marker_f_world;
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('c', icam),
                                                                 gtsam::Symbol('m', imar),
                                                                 marker_f_camera, marker_noise);
      }
    }

    // Create the initial values
    gtsam::Values initial;
    for (size_t icam = 0; icam < sfm_model.cameras_.pose_f_worlds_.size(); icam += 1) {
//      initial.insert(gtsam::Symbol('c', icam), sfm_model.cameras_.pose_f_worlds_[icam]);
//      initial.insert(gtsam::Symbol('c', icam), gtsam::Pose3{});
//      initial.insert(gtsam::Symbol('c', icam), sfm_model.cameras_.pose_f_worlds_[0]);
      initial.insert(gtsam::Symbol('c', icam), sfm_model.cameras_.pose_f_worlds_[icam]
        .compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              gtsam::Point3(0.5, -0.10, 0.20))));
    }
    for (size_t imar = 0; imar < sfm_model.markers_.pose_f_worlds_.size(); imar += 1) {
//      initial.insert(gtsam::Symbol('m', imar), sfm_model.markers_.pose_f_worlds_[imar]);
//      initial.insert(gtsam::Symbol('m', imar), gtsam::Pose3{});
//      initial.insert(gtsam::Symbol('m', imar), sfm_model.markers_.pose_f_worlds_[0]);
      initial.insert(gtsam::Symbol('m', imar), sfm_model.markers_.pose_f_worlds_[imar]
        .compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              gtsam::Point3(0.5 * imar, -0.10, 0.20))));
    }

    /* Optimize the graph and print results */
    gtsam::DoglegParams params{};
    params.verbosity = gtsam::NonlinearOptimizerParams::Verbosity::TERMINATION;
//    params.verbosityDL = gtsam::DoglegParams::VERBOSE;
    gtsam::Values result = gtsam::DoglegOptimizer(graph, initial, params).optimize();
    result.print("Final results:\n");
    std::cout << "initial error = " << graph.error(initial) << std::endl;
    std::cout << "final error = " << graph.error(result) << std::endl;

  }

  int sfm_run()
  {
    SfmModel sfm_model{MarkersConfigurations::square_around_origin_xy_plane,
                       CamerasConfigurations::fly_to_plus_y};

    std::cout << sfm_model.cameras_.pose_f_worlds_[0].rotation().xyz() << std::endl;
    std::cout << sfm_model.cameras_.pose_f_worlds_[0].rotation().ypr() << std::endl;

    simple_sfm(sfm_model);

    return EXIT_SUCCESS;
  }
}

int main()
{
//  return camsim::sfm_gtsam_slam_example();
//  return camsim::sfm_gtsam_example();
//  return camsim::sfm_isam_example();
    return camsim::sfm_run();
}
