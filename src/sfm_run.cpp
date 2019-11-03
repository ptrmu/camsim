
#include "sfm_run.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "sfm_model.hpp"
#include "sfm_resectioning.hpp"

namespace camsim
{
  static void simple_sfm(const SfmModel &sfm_model)
  {
    gtsam::NonlinearFactorGraph graph;

    // Add a prior factor on the first marker.
    // 10cm std on x,y,z 0.3 rad on roll,pitch,yaw
    auto marker_noise = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.3, 0.3, 0.3, 0.1, 0.1, 0.1).finished());
    auto prior_marker_f_world = sfm_model.markers_.markers_[0].pose_f_world_;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(gtsam::Symbol('m', 0),
                                                            prior_marker_f_world,
                                                            marker_noise);

    // Add the between factors
    for (auto &per_camera : sfm_model.corners_f_images_) {
      for (auto &per_marker : per_camera) {
        gtsam::Pose3 marker_f_camera =
          sfm_model.cameras_.cameras_[per_marker.camera_idx_].pose_f_world_.inverse() *
          sfm_model.markers_.markers_[per_marker.marker_idx_].pose_f_world_;
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(gtsam::Symbol('c', per_marker.camera_idx_),
                                                                 gtsam::Symbol('m', per_marker.marker_idx_),
                                                                 marker_f_camera, marker_noise);

      }
    }

    // Create the initial values
    gtsam::Values initial;
    for (auto &camera : sfm_model.cameras_.cameras_) {
//      initial.insert(gtsam::Symbol('c', icam), sfm_model.cameras_.pose_f_worlds_[icam]);
//      initial.insert(gtsam::Symbol('c', icam), gtsam::Pose3{});
//      initial.insert(gtsam::Symbol('c', icam), sfm_model.cameras_.pose_f_worlds_[0]);
      initial.insert(gtsam::Symbol('c', camera.camera_idx_), camera.pose_f_world_
        .compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              gtsam::Point3(0.5, -0.10, 0.20))));
    }
    for (auto &marker : sfm_model.markers_.markers_) {
//      initial.insert(gtsam::Symbol('m', imar), sfm_model.markers_.pose_f_worlds_[imar]);
//      initial.insert(gtsam::Symbol('m', imar), gtsam::Pose3{});
//      initial.insert(gtsam::Symbol('m', imar), sfm_model.markers_.pose_f_worlds_[0]);
      initial.insert(gtsam::Symbol('m', marker.marker_idx_), marker.pose_f_world_
        .compose(gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25),
                              gtsam::Point3(0.5 * marker.marker_idx_, -0.10, 0.20))));
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

    std::cout << sfm_model.cameras_.cameras_[0].pose_f_world_.rotation().xyz() << std::endl;
    std::cout << sfm_model.cameras_.cameras_[0].pose_f_world_.rotation().ypr() << std::endl;

    simple_sfm(sfm_model);

    return EXIT_SUCCESS;
  }

  int sfm_run_resectioning()
  {
    SfmModel sfm_model{MarkersConfigurations::square_around_origin_xy_plane,
                       CamerasConfigurations::fly_to_plus_y};

    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.5, 0.5));

    for (auto &camera : sfm_model.cameras_.cameras_) {
      std::cout << "camera " << camera.camera_idx_ << std::endl;

      CalcCameraPose ccp{sfm_model.cameras_.calibration_,
                         measurement_noise,
                         sfm_model.markers_.corners_f_marker_};

      for (auto &marker : sfm_model.markers_.markers_) {

        auto &corners_f_image = sfm_model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;

        // If the marker was not visible in the image then, obviously, a pose calculation can not be done.
        if (corners_f_image.empty()) {
          std::cout << "Marker not visible" << std::endl;
          continue;
        }

        // Find the camera pose in the marker frame using the GTSAM library
        auto camera_f_marker = ccp.camera_f_marker(corners_f_image);

        // Test that the calculated pose is the same as the original model.
        if (!std::get<0>(camera_f_marker).equals(
          marker.pose_f_world_.inverse() * camera.pose_f_world_)) {
          std::cout << "calculated pose does not match the ground truth" << std::endl;
        }

        // Output the resulting pose and covariance
        std::cout << sfm_model.to_str(camera_f_marker) << std::endl;
      }
    }

    return EXIT_SUCCESS;
  }
}

int main()
{
//  return camsim::sfm_gtsam_slam_example();
//  return camsim::sfm_gtsam_example();
//  return camsim::sfm_isam_example();
//  return camsim::sfm_run();
  return camsim::sfm_run_resectioning();
}
