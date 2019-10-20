
#include "pfm_run.hpp"

#include "pfm_model.hpp"

namespace camsim
{

  enum ModelTypes
  {
    like_gtsam_resection = 0,
    shift_camera_in_z,
    shift_marker_in_z,
    shift_marker_in_x,
    small_rotate_camera__about_x,
  };

  std::unique_ptr<PfmModel> get_model(ModelTypes model_type)
  {
    gtsam::Cal3_S2 camera_calibration{1, 1, 0, 50, 50};

    gtsam::Pose3 camera_f_world{gtsam::Rot3::RzRyRx(M_PI, 0, -M_PI_2),
                                gtsam::Point3(0, 0, 2)};
    gtsam::Pose3 marker_f_world{gtsam::Rot3::RzRyRx(0, 0, 0),
                                gtsam::Point3(0, 0, 0)};

    double marker_size = 20.0;

    switch (model_type) {
      case ModelTypes::like_gtsam_resection:
        break;

      case ModelTypes::shift_camera_in_z:
        camera_f_world = gtsam::Pose3{camera_f_world.rotation(),
                                      camera_f_world.translation() + gtsam::Point3{0, 0, 1}};
        break;

      case ModelTypes::shift_marker_in_z:
        marker_f_world = gtsam::Pose3{marker_f_world.rotation(),
                                      marker_f_world.translation() + gtsam::Point3{0, 0, -2.5}};
        break;

      case ModelTypes::shift_marker_in_x:
        marker_f_world = gtsam::Pose3{marker_f_world.rotation(),
                                      marker_f_world.translation() + gtsam::Point3{2, 0, 0}};
        break;

      case ModelTypes::small_rotate_camera__about_x:
        camera_f_world = gtsam::Pose3{camera_f_world.rotation() * gtsam::Rot3::Rx(0.05),
                                      camera_f_world.translation()};
        break;
    }

    return std::make_unique<PfmModel>(camera_calibration,
                                      camera_f_world,
                                      marker_f_world,
                                      marker_size);
  }

  static int pfm_run()
  {
    auto model = get_model(ModelTypes::shift_marker_in_z);
    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(.5, .5));

    // Get gtsam results
    gtsam::Pose3 gtsam_camera_f_world;
    gtsam::Matrix gtsam_camera_f_world_covariance;

    pfm_gtsam_resection(*model, measurement_noise,
                        gtsam_camera_f_world, gtsam_camera_f_world_covariance);

    std::cout.precision(3);
    std::cout << "gtsam_camera_f_world:\n" << gtsam_camera_f_world << std::endl;
    std::cout << "gtsam_camera_f_world_covariance:\n" << gtsam_camera_f_world_covariance << std::endl << std::endl;
    std::cout << "gtsam_camera_f_world logmap:\n" << gtsam_camera_f_world.Logmap(gtsam_camera_f_world) << std::endl;

    // Get opencv results
    gtsam::Pose3 opencv_camera_f_marker;
    gtsam::Matrix opencv_camera_f_marker_covariance;

    pfm_opencv_resection(*model, measurement_noise,
                         opencv_camera_f_marker, opencv_camera_f_marker_covariance);

    std::cout.precision(3);
    std::cout << "opencv_camera_f_marker:\n" << opencv_camera_f_marker << std::endl;
    std::cout << "opencv_camera_f_marker_covariance:\n" << opencv_camera_f_marker_covariance << std::endl << std::endl;

    return EXIT_SUCCESS;
  }
}

int main()
{
  return camsim::pfm_run();
}
