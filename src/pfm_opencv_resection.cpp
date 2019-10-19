
#include <opencv2/opencv.hpp>
#include "pfm_run.hpp"
#include "pfm_model.hpp"
#include <random>

namespace camsim
{

  static int constexpr num_samples = 5;

  static std::vector<std::vector<cv::Point2d>> generate_normal_corner_image_samples(
    const PfmModel &pfm_model, const gtsam::SharedNoiseModel &measurement_noise)
  {
    // Allocate normal random number generators
    std::random_device rd{};
    std::mt19937 gen{rd()};

    // Create a distribution for each point coordinate
    std::vector<std::pair<std::normal_distribution<>, std::normal_distribution<>>> dists{};
    for (auto &corner_f_image : pfm_model.corners_f_image_) {
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

  void pfm_opencv_resection(const PfmModel &pfm_model, const gtsam::SharedNoiseModel &measurement_noise,
                            gtsam::Pose3 &camera_f_world, gtsam::Matrix &camera_f_world_covariance)
  {
    auto t = generate_normal_corner_image_samples(pfm_model, measurement_noise);
    for (auto &t1 : t) {
      for (auto &t2 : t1) {
        std::cout << std::cout.precision(4) << t2 << ", ";
      }
      std::cout << std::endl;
    }

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3)
      << pfm_model.camera_calibration_.fx(), 0, pfm_model.camera_calibration_.px(),
      0, pfm_model.camera_calibration_.fy(), pfm_model.camera_calibration_.py(),
      0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

    std::vector<cv::Point2d> image_points;
    std::vector<cv::Point3d> world_points;
    for (int i = 0; i < pfm_model.corners_f_world_.size(); i += 1) {
      image_points.emplace_back(cv::Point2d(pfm_model.corners_f_image_[i].x(),
                                            pfm_model.corners_f_image_[i].y()));
      world_points.emplace_back(cv::Point3d(pfm_model.corners_f_world_[i].x(),
                                            pfm_model.corners_f_world_[i].y(),
                                            pfm_model.corners_f_world_[i].z()));
    }

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
    camera_f_world = t_camera_world.inverse();
  }
}
