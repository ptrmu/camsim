
#include "pfm_model.hpp"

namespace camsim
{

  PfmModel::PfmModel(const gtsam::Cal3_S2 &camera_calibration,
                     const gtsam::Pose3 &camera_f_world,
                     const gtsam::Pose3 &marker_f_world,
                     const double &marker_size) :
    camera_calibration_{camera_calibration},
    camera_f_world_{camera_f_world},
    marker_f_world_{marker_f_world},
    marker_size_{marker_size},
    camera_f_marker_{marker_f_world.inverse() * camera_f_world},
    camera_{camera_f_marker_, camera_calibration},
    corners_f_world_{marker_f_world * gtsam::Point3{marker_size / 2, marker_size / 2, 0},
                     marker_f_world * gtsam::Point3{marker_size / 2, -marker_size / 2, 0},
                     marker_f_world * gtsam::Point3{-marker_size / 2, -marker_size / 2, 0},
                     marker_f_world * gtsam::Point3{-marker_size / 2, marker_size / 2, 0}},
    corners_f_image_{camera_.project(corners_f_world_[0]),
                     camera_.project(corners_f_world_[1]),
                     camera_.project(corners_f_world_[2]),
                     camera_.project(corners_f_world_[3])}
  {

  }

  int pfm_model_test()
  {
    gtsam::Cal3_S2 camera_calibration{1, 1, 0, 50, 50};

    gtsam::Pose3 camera_f_world{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                gtsam::Point3(0, 0, 2)};
    gtsam::Pose3 marker_f_world{gtsam::Rot3::RzRyRx(0, 0, 0),
                                gtsam::Point3(0, 0, 0)};

    double marker_size = 20.0;

    PfmModel model{camera_calibration,
                   camera_f_world,
                   marker_f_world,
                   marker_size};

    std::cout << model.corners_f_world_[0]
              << model.corners_f_world_[1]
              << model.corners_f_world_[2]
              << model.corners_f_world_[3]
              << std::endl;

    std::cout << model.corners_f_image_[0]
              << model.corners_f_image_[1]
              << model.corners_f_image_[2]
              << model.corners_f_image_[3]
              << std::endl;

    return EXIT_SUCCESS;
  }
}
