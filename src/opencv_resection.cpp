
#include "camsim.hpp"

#include <opencv2/opencv.hpp>

int opencv_resection()
{

  // 2D image points.
  std::vector<cv::Point2d> image_points;
  image_points.push_back(cv::Point2d(55, 45));
  image_points.push_back(cv::Point2d(45, 45));
  image_points.push_back(cv::Point2d(45, 55));
  image_points.push_back(cv::Point2d(55, 55));

  // 3D model points.
  std::vector<cv::Point3d> model_points;
  model_points.push_back(cv::Point3d(10.0f, 10.0f, 0.0f));
  model_points.push_back(cv::Point3d(-10.0f, 10.0f, 0.0f));
  model_points.push_back(cv::Point3d(-10.0f, -10.0f, 0.0f));
  model_points.push_back(cv::Point3d(10.0f, -10.0f, 0.0f));

  // Camera internals
  double focal_length = 1.0; // Approximate focal length.
  auto center = cv::Point2d(50, 50);
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

  // Output rotation and translation
  cv::Mat rotation_vector; // Rotation in axis-angle form
  cv::Mat translation_vector;

  // Solve for pose
  cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

  std::cout << "Translation Vector" << std::endl << translation_vector << std::endl;
  std::cout << "Rotation Vector " << std::endl << rotation_vector << std::endl;

  cv::Mat rotation_matrix;

  cv::Rodrigues(rotation_vector, rotation_matrix);

  std::cout << "Rotation matrix " << std::endl << rotation_matrix << std::endl;
  return 0;
}
