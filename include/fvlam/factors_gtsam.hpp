#pragma once
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "fvlam/logger.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace fvlam
{

// ==============================================================================
// GtsamUtil class
// ==============================================================================

  struct GtsamUtil
  {
    using Pose3CovarianceMatrixGtsam = Eigen::Matrix<double, gtsam::Pose3::dimension, gtsam::Pose3::dimension>;
    using Pose3CovarianceMatrixRos = Eigen::Matrix<double, 6, 6>;

    // ROS2 has the following definition of a pose transform:
    //  # Row-major representation of the 6x6 covariance matrix (ie values in a row are stored adjacently)
    //  # The orientation parameters use a fixed-axis representation.
    //  # In order, the parameters are:
    //  # (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    //  float64[36] covariance
    // GTSAM has the following definition of a pose transform
    //  (as per f dellaert https://groups.google.com/g/gtsam-users/c/jpz10n7VHqU/m/3IzzrhuQBAAJ)
    //  In GTSAM, a pose is a (Rot3,Point3) pair, in that order. To convert from ROS [TT, TR; RT, RR] you just
    //  form [RR, RT; TR, TT], where each is a 3*3 block.
    //
    // These covariance definitions still do not specify enough to convert covariances between ROS2 and GTSAM.
    // I did a number of experiments where I would generate a covariance in gtsam, convert it to a PoseWithCovariance
    // ROS2 message, and display it in rviz2. This is what I think I figured out:
    //  1) In GTSAM the covariance is expressed in the body frame of the t_world_body transform while in ROS2
    //    the rotation part of the covariance is in the body frame but the translation part is in the world frame.
    //  2) Both ROS2 and GTSAM use rotations about the three (x, y, z) axes to represent a 3D rotation but ROS
    //    seems to use fixed/static axes while GTSAM uses relative axes. One switches the order of the variables
    //    to generate equivalent rotations in the two systems.
    static Pose3CovarianceMatrixGtsam cov_gtsam_from_ros(const gtsam::Pose3 &pose_gtsam,
                                                         const Pose3CovarianceMatrixRos &cov_ros)
    {
      Pose3CovarianceMatrixGtsam cov_gtsam_f_world;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {5, 4, 3, 0, 1, 2};
          cov_gtsam_f_world(r, c) = cov_ros(ro[r], ro[c]);
        }
      }

      // Rotate the translation part of the covariance from the world frame to the body frame.
      Pose3CovarianceMatrixGtsam rotate_translate_part;
      rotate_translate_part << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose_gtsam.rotation().matrix();
      return rotate_translate_part.transpose() * cov_gtsam_f_world * rotate_translate_part;
    }

    static Pose3CovarianceMatrixRos cov_ros_from_gtsam(const gtsam::Pose3 &pose_gtsam,
                                                       const Pose3CovarianceMatrixGtsam &cov_gtsam)
    {
      // Rotate the translation part of the covariance from the body frame to the world frame.
      Pose3CovarianceMatrixRos rotate_translate_part;
      rotate_translate_part << gtsam::I_3x3, gtsam::Z_3x3, gtsam::Z_3x3, pose_gtsam.rotation().matrix();
      gtsam::Pose3::Jacobian cov_gtsam_f_world = rotate_translate_part * cov_gtsam * rotate_translate_part.transpose();

      // interchange the rotate and translation variables and reverse the order of the rotation variables.
      Pose3CovarianceMatrixRos cov_ros;
      for (int r = 0; r < 6; r += 1) {
        for (int c = 0; c < 6; c += 1) {
          static int ro[] = {3, 4, 5, 2, 1, 0};
          cov_ros(r, c) = cov_gtsam_f_world(ro[r], ro[c]);
        }
      }
      return cov_ros;
    }

    static gtsam::Marginals construct_marginals(const gtsam::NonlinearFactorGraph &graph,
                                                const gtsam::Values &result)
    {
      try {
        gtsam::Marginals marginals{graph, result};
        return marginals;
      } catch (gtsam::IndeterminantLinearSystemException &ex) {
        try {
          gtsam::Marginals marginals{graph, result, gtsam::Marginals::QR};
          return marginals;
        } catch (gtsam::IndeterminantLinearSystemException &ex) {
        }
      }
      return gtsam::Marginals{};
    }

    static Transform3WithCovariance extract_transform3_with_covariance(const gtsam::Marginals &marginals,
                                                                       const gtsam::Values &result,
                                                                       gtsam::Key key, bool invert = false)
    {
      auto pose = result.at<gtsam::Pose3>(key);
      auto cov = static_cast<gtsam::Matrix6>(marginals.marginalCovariance(key));

      if (invert) {
        // cov inverse formula from Matías Mattamala: Handling Uncertainty in Estimation Problems with Lie Groups
        // https://docs.google.com/presentation/d/1zLGS3Nr9o9jTAfhY68j5BaiM2V8mSoqjQyNzRN5oG70/edit#slide=id.g8caa8b3668_0_0
        auto adjoint_map = pose.AdjointMap();
        cov = adjoint_map * cov * adjoint_map.transpose();
        pose = pose.inverse();
      }

      return Transform3WithCovariance{Transform3::from(pose), cov_ros_from_gtsam(pose, cov)};
    }

    static Transform3WithCovariance extract_transform3_with_covariance(const gtsam::NonlinearFactorGraph &graph,
                                                                       const gtsam::Values &result,
                                                                       gtsam::Key key, bool invert = false)
    {
      auto marginals{construct_marginals(graph, result)};
      return extract_transform3_with_covariance(marginals, result, key, invert);
    }

  };

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    gtsam::Point2 corner_f_image_;
    gtsam::Key key_marker_;
    gtsam::Key key_camera_;
    gtsam::Point3 corner_f_marker_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    ProjectBetweenFactor(gtsam::Point2 corner_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Key key_marker,
                         gtsam::Key key_camera,
                         gtsam::Point3 corner_f_marker,
                         std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                         Logger &logger,
                         bool throwCheirality = false) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key_marker, key_camera),
      corner_f_image_{std::move(corner_f_image)},
      key_marker_{key_marker}, key_camera_{key_camera},
      corner_f_marker_{std::move(corner_f_marker)},
      cal3ds2_{cal3ds2},
      logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// shorthand for this class
    typedef ProjectBetweenFactor This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &marker_f_world,
                                const gtsam::Pose3 &camera_f_world,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      gtsam::Matrix36 d_point3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;
      gtsam::Matrix23 d_point2_wrt_point3;

      // Transform the point from the Marker frame to the World frame
      gtsam::Point3 point_f_world = marker_f_world.transformFrom(
        corner_f_marker_,
        H1 ? gtsam::OptionalJacobian<3, 6>(d_point3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_world, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_world,
          H2 ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none,
          H1 ? gtsam::OptionalJacobian<2, 3>(d_point2_wrt_point3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_point3 * d_point3_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3;
        }

        // Return the error.
        return point_f_image - corner_f_image_;

      } catch (gtsam::CheiralityException &e) {
        if (H1) *H1 = gtsam::Matrix26::Zero();
        if (H2) *H2 = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": Marker " << gtsam::DefaultKeyFormatter(key_marker_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };

// ==============================================================================
// ResectioningFactor class
// ==============================================================================

  class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    gtsam::Key key_camera_;
    const gtsam::Point2 corner_f_image;
    const gtsam::Point3 corner_f_world;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(gtsam::Key key_camera,
                       gtsam::Point2 corner_f_image,
                       const gtsam::SharedNoiseModel &model,
                       gtsam::Point3 corner_f_world,
                       std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                       Logger &logger,
                       bool throwCheirality = false) :
      NoiseModelFactor1<gtsam::Pose3>(model, key_camera),
      key_camera_{key_camera},
      corner_f_image{std::move(corner_f_image)},
      corner_f_world{std::move(corner_f_world)},
      cal3ds2_{std::move(cal3ds2)},
      logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// shorthand for this class
    typedef ResectioningFactor This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, *cal3ds2_};
      try {
        return camera.project(corner_f_world, H) - corner_f_image;
      } catch (gtsam::CheiralityException &e) {
        if (H) *H = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": point moved behind camera "
                        << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };

// ==============================================================================
// QuadResectioningFactor class
// ==============================================================================

// This is just like the Resectioning factor except that it takes all 4 corners at once.
// This might be a little faster but mostly it was an experiment to create another
// custom factor.
#if 0
  class QuadResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    const std::vector<gtsam::Point2> points_f_image;
    gtsam::Key key_camera_;
    const std::vector<gtsam::Point3> points_f_world;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    /// Construct factor given known point P and its projection p
    QuadResectioningFactor(std::vector<gtsam::Point2> points_f_image,
                           const gtsam::SharedNoiseModel &model,
                           gtsam::Key key_camera,
                           std::vector<gtsam::Point3> points_f_world,
                           std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
                           Logger &logger,
                           bool throwCheirality = false) :
      NoiseModelFactor1<gtsam::Pose3>(model, key_camera),
      points_f_image{std::move(points_f_image)},
      key_camera_{key_camera},
      points_f_world{std::move(points_f_world)},
      cal3ds2_{cal3ds2},
      logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// shorthand for this class
    typedef QuadResectioningFactor This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, *cal3ds2_};
      try {
        gtsam::Matrix26 H0, H1, H2, H3;
        gtsam::Vector2 e0 = camera.project(points_f_world[0], H0) - points_f_image[0];
        gtsam::Vector2 e1 = camera.project(points_f_world[1], H1) - points_f_image[1];
        gtsam::Vector2 e2 = camera.project(points_f_world[2], H2) - points_f_image[2];
        gtsam::Vector2 e3 = camera.project(points_f_world[3], H3) - points_f_image[3];

        if (H) {
          *H = (gtsam::Matrix86{} << H0, H1, H2, H3).finished();
        }

        return (gtsam::Vector8{} << e0, e1, e2, e3).finished();

      } catch (gtsam::CheiralityException &e) {
        if (H) *H = gtsam::Matrix86::Zero();

        logger_.error() << e.what() << ": point moved behind camera "
                        << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      auto max_e = gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
      return (gtsam::Vector8{} << max_e, max_e, max_e, max_e).finished();
    }
  };
#endif

// ==============================================================================
// QuadResectioningFactor class
// ==============================================================================

  class QuadResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    gtsam::Key key_camera_;
    const std::vector<gtsam::Point2> corners_f_image_;
    const std::vector<gtsam::Point3> corners_f_world_;
    bool use_transform_;
    gtsam::Pose3 t_camera_imager_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    Logger &logger_;
    std::string debug_str_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)

  public:
    /// Construct factor given known point P and its projection p
    QuadResectioningFactor(gtsam::Key key_camera,
                           std::vector<gtsam::Point2> corners_f_image,
                           const gtsam::SharedNoiseModel &model,
                           std::vector<gtsam::Point3> corners_f_world,
                           bool use_transform,
                           gtsam::Pose3 t_camera_imager,
                           std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
                           Logger &logger,
                           std::string debug_str,
                           bool throwCheirality = false) :
      NoiseModelFactor1<gtsam::Pose3>(model, key_camera),
      key_camera_{key_camera},
      corners_f_image_{std::move(corners_f_image)},
      corners_f_world_{std::move(corners_f_world)},
      use_transform_{use_transform},
      t_camera_imager_{std::move(t_camera_imager)},
      cal3ds2_{cal3ds2},
      logger_{logger},
      debug_str_{debug_str},
      throwCheirality_{throwCheirality}
    {}

    /// shorthand for this class
    typedef QuadResectioningFactor This;

    /// shorthand for a smart pointer to a factor
    typedef boost::shared_ptr<This> shared_ptr;

    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H = boost::none) const override
    {
#if 1
      gtsam::Matrix66 HT;
      auto camera_pose = use_transform_ ?
                         pose.compose(t_camera_imager_, (H) ? gtsam::OptionalJacobian(HT) : boost::none) :
                         pose;

      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_pose, *cal3ds2_};

      try {
        gtsam::Matrix26 H0, H1, H2, H3;
        auto e = (gtsam::Vector8{}
          << camera.project(corners_f_world_[0], (H) ? gtsam::OptionalJacobian(H0) : boost::none) - corners_f_image_[0],
          camera.project(corners_f_world_[1], (H) ? gtsam::OptionalJacobian(H1) : boost::none) - corners_f_image_[1],
          camera.project(corners_f_world_[2], (H) ? gtsam::OptionalJacobian(H2) : boost::none) - corners_f_image_[2],
          camera.project(corners_f_world_[3], (H) ? gtsam::OptionalJacobian(H3) : boost::none) - corners_f_image_[3]
        ).finished();

        if (H) {
          if (use_transform_) {
            H0 = H0 * HT;
            H1 = H1 * HT;
            H2 = H2 * HT;
            H3 = H3 * HT;
          }
          *H = (gtsam::Matrix86{} << H0, H1, H2, H3).finished();
        }
        return e;

      } catch (gtsam::CheiralityException &e) {
        if (H) *H = gtsam::Matrix86::Zero();

        if (!debug_str_.empty()) {
          logger_.error() << e.what() << ": " << debug_str_ << " point moved behind camera "
                          << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;
        }

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      auto max_e = gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
      return (gtsam::Vector8{} << max_e, max_e, max_e, max_e).finished();

#else
      try {
        if (!use_transform_) {
          auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, *cal3ds2_};
          if (!H) {
            return (gtsam::Vector8{}
              << camera.project(points_f_world_[0]) - points_f_image_[0],
              camera.project(points_f_world_[1]) - points_f_image_[1],
              camera.project(points_f_world_[2]) - points_f_image_[2],
              camera.project(points_f_world_[3]) - points_f_image_[3]).finished();
          }

          gtsam::Matrix26 H0, H1, H2, H3;
          gtsam::Vector2 e0 = camera.project(points_f_world_[0], H0) - points_f_image_[0];
          gtsam::Vector2 e1 = camera.project(points_f_world_[1], H1) - points_f_image_[1];
          gtsam::Vector2 e2 = camera.project(points_f_world_[2], H2) - points_f_image_[2];
          gtsam::Vector2 e3 = camera.project(points_f_world_[3], H3) - points_f_image_[3];

          *H = (gtsam::Matrix86{} << H0, H1, H2, H3).finished();
          return (gtsam::Vector8{} << e0, e1, e2, e3).finished();
        }

        if (!H) {
          auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose.compose(t_camera_imager_), *cal3ds2_};
          return (gtsam::Vector8{}
            << camera.project(points_f_world_[0]) - points_f_image_[0],
            camera.project(points_f_world_[1]) - points_f_image_[1],
            camera.project(points_f_world_[2]) - points_f_image_[2],
            camera.project(points_f_world_[3]) - points_f_image_[3]).finished();
        }

        gtsam::Matrix HT;
        auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose.compose(t_camera_imager_, HT), *cal3ds2_};
        gtsam::Matrix26 H0, H1, H2, H3;
        gtsam::Vector2 e0 = camera.project(points_f_world_[0], H0) - points_f_image_[0];
        gtsam::Vector2 e1 = camera.project(points_f_world_[1], H1) - points_f_image_[1];
        gtsam::Vector2 e2 = camera.project(points_f_world_[2], H2) - points_f_image_[2];
        gtsam::Vector2 e3 = camera.project(points_f_world_[3], H3) - points_f_image_[3];

        *H = (gtsam::Matrix86{} << H0 * HT, H1 * HT, H2 * HT, H3 * HT).finished();
        return (gtsam::Vector8{} << e0, e1, e2, e3).finished();

      } catch (gtsam::CheiralityException &e) {
        if (H) *H = gtsam::Matrix86::Zero();

        if (!debug_str_.empty()) {
          logger_.error() << e.what() << ": " << debug_str_ << " point moved behind camera "
                          << gtsam::DefaultKeyFormatter(key_camera_) << std::endl;
        }

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_camera_);
      }
      auto max_e = gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
      return (gtsam::Vector8{} << max_e, max_e, max_e, max_e).finished();
#endif
    }
  };
}
