
#include "sfm_model.hpp"

namespace camsim
{
  static std::vector<gtsam::SimpleCamera> gen_cameras(const gtsam::Cal3_S2 &camera_calibration,
                                                      const std::vector<gtsam::Pose3> &camera_f_worlds)
  {
    std::vector<gtsam::SimpleCamera> cameras;
    for (auto camera_f_world : camera_f_worlds) {
      cameras.emplace_back(gtsam::SimpleCamera{camera_f_world, camera_calibration});
    }
    return cameras;
  }

  static std::vector<std::vector<gtsam::Point3>> gen_corners_f_worlds(const std::vector<gtsam::Pose3> &marker_f_worlds,
                                                                      const double &marker_size)
  {
    std::vector<std::vector<gtsam::Point3>> corners_f_worlds;
    for (auto marker_f_world : marker_f_worlds) {
      corners_f_worlds.emplace_back(std::vector<gtsam::Point3>
                                      {marker_f_world * gtsam::Point3{marker_size / 2, marker_size / 2, 0},
                                       marker_f_world * gtsam::Point3{marker_size / 2, -marker_size / 2, 0},
                                       marker_f_world * gtsam::Point3{-marker_size / 2, -marker_size / 2, 0},
                                       marker_f_world * gtsam::Point3{-marker_size / 2, marker_size / 2, 0}}
      );
    }
    return corners_f_worlds;
  }

  static std::vector<std::vector<std::vector<gtsam::Point2>>> gen_corners_f_images(
    const std::vector<gtsam::SimpleCamera> cameras,
    std::vector<std::vector<gtsam::Point3>> corners_f_worlds)
  {
    std::vector<std::vector<std::vector<gtsam::Point2>>> corners_f_images;
    for (auto camera : cameras) {
      std::vector<std::vector<gtsam::Point2>> per_camera;
      for (auto corners_f_world : corners_f_worlds) {
        per_camera.emplace_back(std::vector<gtsam::Point2>{camera.project(corners_f_world[0]),
                                                           camera.project(corners_f_world[1]),
                                                           camera.project(corners_f_world[2]),
                                                           camera.project(corners_f_world[3])});
      }
      corners_f_images.emplace_back(per_camera);
    }
    return corners_f_images;
  }

  SfmModel::SfmModel(const gtsam::Cal3_S2 &camera_calibration,
                     const std::vector<gtsam::Pose3> &camera_f_worlds,
                     const std::vector<gtsam::Pose3> &marker_f_worlds,
                     const double &marker_size) :
    camera_calibration_{camera_calibration},
    camera_f_worlds_{camera_f_worlds},
    marker_f_worlds_{marker_f_worlds},
    marker_size_{marker_size},
    cameras_{gen_cameras(camera_calibration, camera_f_worlds)},
    corners_f_worlds_{gen_corners_f_worlds(marker_f_worlds, marker_size)},
    corners_f_images_{gen_corners_f_images(cameras_, corners_f_worlds_)}
  {

  }


}