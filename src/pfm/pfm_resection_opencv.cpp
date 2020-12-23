
#include <math.h>
#include <opencv2/opencv.hpp>
#include "pfm_run.hpp"
#include "../pose_with_covariance.hpp"
#include <random>

namespace camsim
{

  static int constexpr num_samples = 4096;

  static std::vector<std::vector<cv::Point2d>> generate_normal_corners_f_images(
    const std::vector<gtsam::Point2> &corners_f_image,
    const gtsam::SharedNoiseModel &measurement_noise)
  {
    // Allocate normal random number generators
//    std::random_device rd{};
//    std::mt19937 gen{rd()};
    std::mt19937 gen{8};

    // Create a distribution for each point coordinate
    std::vector<std::pair<std::normal_distribution<>, std::normal_distribution<>>> dists{};
    for (auto &corner_f_image : corners_f_image) {
      dists.emplace_back(std::pair<std::normal_distribution<>, std::normal_distribution<>>{
        std::normal_distribution<>{corner_f_image.x(), measurement_noise->sigmas()(0)},
        std::normal_distribution<>{corner_f_image.y(), measurement_noise->sigmas()(1)}
      });
    }

    // Generate a lot of samples
    std::vector<std::vector<cv::Point2d>> image_points_samples(num_samples);
    for (auto &image_points_sample : image_points_samples) {
      std::vector<cv::Point2d> sample{};
      for (auto &dist : dists) {
        sample.emplace_back(cv::Point2d{dist.first(gen), dist.second(gen)});
      }
      image_points_sample = std::move(sample);
    }

    return image_points_samples;
  }

  static std::vector<gtsam::Pose3> many_opencv_resection(const gtsam::Cal3_S2 &camera_calibration,
                                                         const std::vector<gtsam::Point3> &corners_f_world,
                                                         const std::vector<std::vector<cv::Point2d>> &corners_f_images)
  {
    std::vector<gtsam::Pose3> camera_f_worlds{};

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
      << camera_calibration.fx(), 0, camera_calibration.px(),
      0, camera_calibration.fy(), camera_calibration.py(),
      0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

    std::vector<cv::Point3d> world_points;
    for (int i = 0; i < corners_f_world.size(); i += 1) {
      world_points.emplace_back(cv::Point3d(corners_f_world[i].x(),
                                            corners_f_world[i].y(),
                                            corners_f_world[i].z()));
    }

    for (auto &image_points : corners_f_images) {
      // Solve for pose
      cv::Mat rotation_vector; // Rotation in axis-angle form
      cv::Mat translation_vector;
      cv::solvePnP(world_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

      // Convert to gtsam pose
      cv::Mat rotation_matrix;
      cv::Rodrigues(rotation_vector, rotation_matrix);

      gtsam::Rot3 rot3{
        rotation_matrix.at<double>(0, 0), rotation_matrix.at<double>(0, 1), rotation_matrix.at<double>(0, 2),
        rotation_matrix.at<double>(1, 0), rotation_matrix.at<double>(1, 1), rotation_matrix.at<double>(1, 2),
        rotation_matrix.at<double>(2, 0), rotation_matrix.at<double>(2, 1), rotation_matrix.at<double>(2, 2)};
      gtsam::Vector3 tran{translation_vector.at<double>(0),
                          translation_vector.at<double>(1),
                          translation_vector.at<double>(2)};

      // This is what the OpenCV docs say about the solvePnp function: "This function returns the rotation and
      // the translation vectors that transform a 3D point expressed in the object coordinate frame to the camera
      // coordinate frame". In our notation, this is T_camera_object. In this particular case, the object frame
      // is the world frame. So SolvePnp returns T_camera_world. We need to invert this to get camera_f_world.
      auto t_camera_world = gtsam::Pose3{rot3, tran};
      camera_f_worlds.emplace_back(t_camera_world.inverse());
    }

    return camera_f_worlds;
  }

  static void mean_camera_f_world(std::vector<gtsam::Pose3> camera_f_worlds,
                                  gtsam::Pose3 &camera_f_world,
                                  gtsam::Matrix6 &camera_f_world_covariance)
  {
    if (camera_f_worlds.empty()) {
      return;
    }
    if (camera_f_worlds.size() < 2) {
      camera_f_world = camera_f_worlds[0];
    }

    // for the mean calculation
    gtsam::Point3 t_mean{};
    gtsam::Vector3 r_mean{gtsam::Z_3x1};
    gtsam::Vector3 r_offset{camera_f_worlds[0].rotation().xyz()};
    for (auto &cam_f_world: camera_f_worlds) {
      t_mean += cam_f_world.translation();
      // Offset an angle away from +-pi.
      gtsam::Vector3 r_tmp = cam_f_world.rotation().xyz();
      r_tmp -= r_offset;
      // Normalize
      r_tmp(0) = r_tmp(0) < -M_PI ? r_tmp(0) + 2 * M_PI : (r_tmp(0) > M_PI ? r_tmp(0) - 2 * M_PI : r_tmp(0));
      r_tmp(1) = r_tmp(1) < -M_PI ? r_tmp(1) + 2 * M_PI : (r_tmp(1) > M_PI ? r_tmp(1) - 2 * M_PI : r_tmp(1));
      r_tmp(2) = r_tmp(2) < -M_PI ? r_tmp(2) + 2 * M_PI : (r_tmp(2) > M_PI ? r_tmp(2) - 2 * M_PI : r_tmp(2));
      r_mean += r_tmp;
    }

    t_mean /= camera_f_worlds.size();
    r_mean /= camera_f_worlds.size();

    // undo offset
    r_mean += r_offset;

    // Normalize
    r_mean(0) = r_mean(0) < -M_PI ? r_mean(0) + 2 * M_PI : r_mean(0) > 2 * M_PI ? r_mean(0) - M_PI : r_mean(0);
    r_mean(1) = r_mean(1) < -M_PI ? r_mean(1) + 2 * M_PI : r_mean(1) > 2 * M_PI ? r_mean(1) - M_PI : r_mean(1);
    r_mean(2) = r_mean(2) < -M_PI ? r_mean(2) + 2 * M_PI : r_mean(2) > 2 * M_PI ? r_mean(2) - M_PI : r_mean(2);

    // Calculate variance
    gtsam::Vector6 mean;
    mean << r_mean(0), r_mean(1), r_mean(2), t_mean(0), t_mean(1), t_mean(2);
    gtsam::Matrix6 cov{gtsam::Z_6x6};
    for (auto &cam_f_world: camera_f_worlds) {
      gtsam::Vector6 res;
      auto xyz = cam_f_world.rotation().xyz();
      auto t = cam_f_world.translation();
      res << xyz(0), xyz(1), xyz(2), t(0), t(1), t(2);
      res -= mean;
      res(0) = res(0) < -M_PI ? res(0) + 2 * M_PI : (res(0) > M_PI ? res(0) - 2 * M_PI : res(0));
      res(1) = res(1) < -M_PI ? res(1) + 2 * M_PI : (res(1) > M_PI ? res(1) - 2 * M_PI : res(1));
      res(2) = res(2) < -M_PI ? res(2) + 2 * M_PI : (res(2) > M_PI ? res(2) - 2 * M_PI : res(2));
      auto cross_res = res * res.transpose();
      cov += cross_res;
    }

    cov /= camera_f_worlds.size();

    camera_f_world = gtsam::Pose3{gtsam::Rot3::RzRyRx(r_mean(0), r_mean(1), r_mean(2)), t_mean};
    camera_f_world_covariance = cov;
  }

  void pfm_resection_opencv(const gtsam::Cal3_S2 &camera_calibration,
                            const std::vector<gtsam::Point2> &corners_f_image,
                            const std::vector<gtsam::Point3> &corners_f_world,
                            const gtsam::Pose3 &camera_f_world_initial,
                            const gtsam::SharedNoiseModel &measurement_noise,
                            gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance)
  {
    auto corners_f_images = generate_normal_corners_f_images(corners_f_image, measurement_noise);
//    std::cout.precision(3);
//    std::cout.setf(std::ios::fixed);
//    for (auto &t1 : corners_f_images) {
//      for (auto &t2 : t1) {
//        std::cout << t2 << ", ";
//      }
//      std::cout << std::endl;
//    }

    auto camera_f_worlds = many_opencv_resection(camera_calibration, corners_f_world, corners_f_images);
//    std::cout.precision(3);
//    std::cout.setf(std::ios::fixed);
//    for (auto &t1 : camera_f_worlds) {
//      std::cout << t1;
//    }

//    for (int i = 0; i < camera_f_worlds.size(); i += 1) {
//      auto &corners_f_image = corners_f_images[i];
//      auto &camera_f_world = camera_f_worlds[i];
//      std::cout << corners_f_image[0]
//                << corners_f_image[1]
//                << corners_f_image[2]
//                << corners_f_image[3] << std::endl;
//      std::cout << "                         " << PoseWithCovariance::to_str(camera_f_world) << std::endl;
//    }

    mean_camera_f_world(std::move(camera_f_worlds), camera_f_world, camera_f_world_covariance);
  }
}
