
#include "sfm_resectioning.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/inference/Symbol.h"
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
    const gtsam::Cal3DS2 &K_;
    const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                      const gtsam::Point3 &,
                                      boost::optional<gtsam::Matrix &>)> &project_func_;
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
      const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                        const gtsam::Point3 &,
                                        boost::optional<gtsam::Matrix &>)> &project_func_;
      const gtsam::Point3 P_;       ///< 3D point on the calibration rig
      const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

    public:
      /// Construct factor given known point P and its projection p
      ResectioningFactor(const gtsam::SharedNoiseModel &model,
                         const gtsam::Key &key,
                         const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                           const gtsam::Point3 &,
                                                           boost::optional<gtsam::Matrix &>)> &project_func,
                         gtsam::Point2 p,
                         gtsam::Point3 P) :
        NoiseModelFactor1<gtsam::Pose3>(model, key),
        project_func_{project_func},
        P_(std::move(P)),
        p_(std::move(p))
      {}

      /// evaluate the error
      gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                  boost::optional<gtsam::Matrix &> H) const override
      {
        return project_func_(pose, P_, H) - p_;
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
    CalcCameraPoseImpl(const gtsam::Cal3DS2 &K,
                       const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                         const gtsam::Point3 &,
                                                         boost::optional<gtsam::Matrix &>)> &project_func,
                       const gtsam::SharedNoiseModel &measurement_noise,
                       const std::vector<gtsam::Point3> &corners_f_marker) :
      K_{K}, project_func_{project_func},
      measurement_noise_{measurement_noise}, corners_f_marker_{corners_f_marker},
      camera_matrix_{(cv::Mat_<double>(3, 3)
        << K.fx(), 0, K.px(),
        0, K.fy(), K.py(),
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

    PoseWithCovariance camera_f_marker(
      int marker_id,
      const std::vector<gtsam::Point2> &corners_f_image)
    {
      graph_.resize(0);
      initial_.clear();

      /* Add measurement factors to the graph */
      for (size_t i = 0; i < corners_f_image.size(); i += 1) {
        graph_.emplace_shared<ResectioningFactor>(measurement_noise_, X1_,
                                                  project_func_,
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

      return PoseWithCovariance{
        static_cast<std::uint64_t>(marker_id),
        camera_f_marker,
        camera_f_marker_covariance};
    }
  };

  CalcCameraPose::CalcCameraPose(const gtsam::Cal3DS2 &K,
                                 const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                                   const gtsam::Point3 &,
                                                                   boost::optional<gtsam::Matrix &>)> &project_func,
                                 const gtsam::SharedNoiseModel &measurement_noise,
                                 const std::vector<gtsam::Point3> &corners_f_marker) :
    impl_{std::make_unique<CalcCameraPoseImpl>(K, project_func, measurement_noise, corners_f_marker)}
  {}

  CalcCameraPose::~CalcCameraPose() = default;

  PoseWithCovariance CalcCameraPose::camera_f_marker(
    int marker_id,
    const std::vector<gtsam::Point2> &corners_f_image)
  {
    return impl_->camera_f_marker(marker_id, corners_f_image);
  }
}


#include "model.hpp"

int sfm_run_resectioning()
{
  camsim::Model model{camsim::MarkersConfigurations::square_around_origin_xy_plane,
                      camsim::CamerasConfigurations::z2_facing_origin,
                      camsim::CameraTypes::distorted_camera};

  auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.5, 0.5));

  for (auto &camera : model.cameras_.cameras_) {
    std::cout << std::endl
              << "************************" << std::endl
              << "camera " << camera.index() << std::endl;

    camsim::CalcCameraPose ccp{model.cameras_.calibration_,
                               model.cameras_.project_func_,
                               measurement_noise,
                               model.markers_.corners_f_marker_};

    for (auto &marker : model.markers_.markers_) {

      auto &corners_f_image = model.corners_f_images_[camera.index()][marker.index()].corners_f_image_;

      // If the marker was not visible in the image then, obviously, a pose calculation can not be done.
      if (corners_f_image.empty()) {
        std::cout << "Marker not visible" << std::endl << std::endl;
        continue;
      }

      // Find the camera pose in the marker frame using the GTSAM library
      auto camera_f_marker = ccp.camera_f_marker(marker.index(), corners_f_image);

      // Test that the calculated pose is the same as the original model.
      if (!camera_f_marker.pose_.equals(
        marker.marker_f_world_.inverse() * camera.camera_f_world_)) {
        std::cout << "calculated pose does not match the ground truth" << std::endl;
      }

      // Output the resulting pose and covariance
      std::cout << camera_f_marker.to_str() << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
