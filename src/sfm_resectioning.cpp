
#include "sfm_resectioning.hpp"

#include "gtsam/inference/Symbol.h"
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <opencv2/opencv.hpp>

namespace camsim
{
  // Use gtsam to calculate a camera's pose in a marker's frame.
  // Use the pose from opencv SolvePnp as the initial estimate.
  class CalcCameraPoseImpl
  {
    const gtsam::Cal3_S2 &K_;
    const gtsam::SharedNoiseModel measurement_noise_; // The structure gets lost if this is a reference (not sure why)
    const std::vector<gtsam::Point3> &corners_f_marker_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_{cv::Mat::zeros(4, 1, cv::DataType<double>::type)}; // Assuming no lens distortion
    std::vector<cv::Point3d> cv_corners_f_marker;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};
    gtsam::Key X1_{gtsam::Symbol('x', 1)};


    class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {
      typedef NoiseModelFactor1 <gtsam::Pose3> Base;

      const gtsam::Cal3_S2 &K_;     ///< camera's intrinsic parameters
      const gtsam::Point3 P_;       ///< 3D point on the calibration rig
      const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

    public:
      /// Construct factor given known point P and its projection p
      ResectioningFactor(const gtsam::SharedNoiseModel &model, const gtsam::Key &key,
                         const gtsam::Cal3_S2 &calib, gtsam::Point2 p, gtsam::Point3 P) :
        Base(model, key), K_(calib), P_(std::move(P)), p_(std::move(p))
      {}

      /// evaluate the error
      gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                  boost::optional<gtsam::Matrix &> H = boost::none) const override
      {
        gtsam::SimpleCamera camera(pose, K_);
        return camera.project(P_, H, boost::none, boost::none) - p_;
      }
    };

    gtsam::Pose3 cv_camera_f_marker(
      const std::vector<gtsam::Point2> &corners_f_image)
    {

      std::vector<cv::Point2d> cv_corners_f_image{
        cv::Point2d{corners_f_image[0].x(), corners_f_image[0].y()},
        cv::Point2d{corners_f_image[1].x(), corners_f_image[1].y()},
        cv::Point2d{corners_f_image[2].x(), corners_f_image[2].y()},
        cv::Point2d{corners_f_image[3].x(), corners_f_image[3].y()}
      };

      // Output rotation and translation
      cv::Mat rotation_vector; // Rotation in axis-angle form
      cv::Mat translation_vector;

      // Solve for camera_f_marker
      cv::solvePnP(cv_corners_f_marker, cv_corners_f_image,
                   camera_matrix_, dist_coeffs_,
                   rotation_vector, translation_vector);

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
      // coordinate frame". In our notation, this is T_camera_object or object_f_camera. In this particular case,
      // the object frame is the world frame. So SolvePnp returns T_camera_marker or marker_f_camera. We need to
      // invert this to get camera_f_marker.
      auto marker_f_camera = gtsam::Pose3{rot3, tran};
      return marker_f_camera.inverse();
    }

  public:
    CalcCameraPoseImpl(const gtsam::Cal3_S2 &K,
                       const gtsam::SharedNoiseModel &measurement_noise,
                       const std::vector<gtsam::Point3> &corners_f_marker) :
      K_{K}, measurement_noise_{measurement_noise}, corners_f_marker_{corners_f_marker},
      camera_matrix_{(cv::Mat_<double>(3, 3)
        << K.fx(), 0, K.principalPoint().x(),
        0, K.fy(), K.principalPoint().y(),
        0, 0, 1)},
      cv_corners_f_marker{cv::Point3d{corners_f_marker[0].x(),
                                      corners_f_marker[0].y(),
                                      corners_f_marker[0].z()},
                          cv::Point3d{corners_f_marker[1].x(),
                                      corners_f_marker[1].y(),
                                      corners_f_marker[1].z()},
                          cv::Point3d{corners_f_marker[2].x(),
                                      corners_f_marker[2].y(),
                                      corners_f_marker[2].z()},
                          cv::Point3d{corners_f_marker[3].x(),
                                      corners_f_marker[3].y(),
                                      corners_f_marker[3].z()}}
    {}

    SfmPoseWithCovariance camera_f_marker(
      int marker_id,
      const std::vector<gtsam::Point2> &corners_f_image)
    {
      graph_.resize(0);
      initial_.clear();

      /* Add measurement factors to the graph */
      for (size_t i = 0; i < corners_f_image.size(); i += 1) {
        graph_.emplace_shared<ResectioningFactor>(measurement_noise_, X1_, K_,
                                                  corners_f_image[i],
                                                  corners_f_marker_[i]);
      }

      /* Create an initial estimate for the camera pose using the opencv SolvePnp routine */
      initial_.insert(X1_, cv_camera_f_marker(corners_f_image));

      /* Optimize the graph using Levenberg-Marquardt*/
      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_).optimize();

      // Get the pose and covariance
      auto camera_f_marker = result.at<gtsam::Pose3>(X1_);
      gtsam::Marginals marginals(graph_, result);
      auto camera_f_marker_covariance = marginals.marginalCovariance(X1_);

      return SfmPoseWithCovariance{
        marker_id,
        result.at<gtsam::Pose3>(X1_),
        marginals.marginalCovariance(X1_)};
    }
  };

  CalcCameraPose::CalcCameraPose(const gtsam::Cal3_S2 &K,
                                 const gtsam::SharedNoiseModel &measurement_noise,
                                 const std::vector<gtsam::Point3> &corners_f_marker) :
    impl_{std::make_unique<CalcCameraPoseImpl>(K, measurement_noise, corners_f_marker)}
  {}

  CalcCameraPose::~CalcCameraPose() = default;

  SfmPoseWithCovariance CalcCameraPose::camera_f_marker(
    int marker_id,
    const std::vector<gtsam::Point2> &corners_f_image)
  {
    return impl_->camera_f_marker(marker_id, corners_f_image);
  }
}


#include "sfm_model.hpp"

int sfm_run_resectioning()
{
  camsim::SfmModel sfm_model{camsim::MarkersConfigurations::square_around_origin_xy_plane,
                             camsim::CamerasConfigurations::fly_to_plus_y};

  auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.5, 0.5));

  for (auto &camera : sfm_model.cameras_.cameras_) {
    std::cout << std::endl
              << "************************" << std::endl
              << "camera " << camera.camera_idx_ << std::endl;

    camsim::CalcCameraPose ccp{sfm_model.cameras_.calibration_,
                               measurement_noise,
                               sfm_model.markers_.corners_f_marker_};

    for (auto &marker : sfm_model.markers_.markers_) {

      auto &corners_f_image = sfm_model.corners_f_images_[camera.camera_idx_][marker.marker_idx_].corners_f_image_;

      // If the marker was not visible in the image then, obviously, a pose calculation can not be done.
      if (corners_f_image.empty()) {
        std::cout << "Marker not visible" << std::endl << std::endl;
        continue;
      }

      // Find the camera pose in the marker frame using the GTSAM library
      auto camera_f_marker = ccp.camera_f_marker(marker.marker_idx_, corners_f_image);

      // Test that the calculated pose is the same as the original model.
      if (!camera_f_marker.pose_.equals(
        marker.pose_f_world_.inverse() * camera.pose_f_world_)) {
        std::cout << "calculated pose does not match the ground truth" << std::endl;
      }

      // Output the resulting pose and covariance
      std::cout << camera_f_marker.to_str() << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
