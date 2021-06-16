#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace camsim
{

// ==============================================================================
// MarkerCornerFactor class
// ==============================================================================

  class MarkerCornerFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
  {
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> Base;
    typedef MarkerCornerFactor This;

    const gtsam::Point3 corner_f_marker_;

  public:

    MarkerCornerFactor(const gtsam::Key &key_pose, const gtsam::Key &key_point,
                       gtsam::Point3 corner_f_marker,
                       const gtsam::SharedNoiseModel &model) :
      Base(model, key_pose, key_point), corner_f_marker_{std::move(corner_f_marker)}
    {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                const gtsam::Point3 &corner_f_world,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      if (H2) {
        (*H2) = -1.0 * gtsam::I_3x3;
      }
      return pose.transformFrom(corner_f_marker_, H1) - corner_f_world;
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }
  };

// ==============================================================================
// Imager0Imager1Factor class
// ==============================================================================

  class Imager0Imager1Factor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
    using This = Imager0Imager1Factor;
    using shared_ptr = boost::shared_ptr<This>;

    gtsam::Key key_t_w_i0_;
    gtsam::Key key_t_i0_i1_;
    gtsam::Point2 i1_corner_f_image_;
    gtsam::Point3 corner_f_marker_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    fvlam::Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)


  public:
    Imager0Imager1Factor(const gtsam::Key &key_t_m_i0, const gtsam::Key &key_t_i0_i1,
                         gtsam::Point2 i1_corner_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Point3 corner_f_marker,
                         std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                         fvlam::Logger &logger,
                         bool throwCheirality = false) :
      Base(model, key_t_m_i0, key_t_i0_i1),
      key_t_w_i0_{key_t_m_i0}, key_t_i0_i1_{key_t_i0_i1},
      i1_corner_f_image_{i1_corner_f_image}, corner_f_marker_{corner_f_marker},
      cal3ds2_{cal3ds2}, logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }


    gtsam::Vector evaluateError(const gtsam::Pose3 &t_m_i0,
                                const gtsam::Pose3 &t_i0_i1,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      gtsam::Matrix66 d_pose3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;

      // Transform the point from the Marker frame to the World frame
      auto t_m_i1 = t_m_i0.compose(
        t_i0_i1,
        H1 ? gtsam::OptionalJacobian<6, 6>(d_pose3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_m_i1, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          corner_f_marker_,
          (H1 || H2) ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_pose3 * d_pose3_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3;
        }

        // Return the error.
        return point_f_image - i1_corner_f_image_;

      } catch (gtsam::CheiralityException &e) {
        if (H1) *H1 = gtsam::Matrix26::Zero();
        if (H2) *H2 = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": t_w_i0 " << gtsam::DefaultKeyFormatter(key_t_w_i0_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_t_i0_i1_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_t_w_i0_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };

// ==============================================================================
// Marker0Marker1Factor class
// ==============================================================================

  class Marker0Marker1Factor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
    using This = Marker0Marker1Factor;
    using shared_ptr = boost::shared_ptr<This>;

    gtsam::Key key_t_m0_c_;
    gtsam::Key key_t_m0_m1_;
    gtsam::Point2 m1_corner_f_image_;
    gtsam::Point3 m1_corner_f_marker_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    fvlam::Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)


  public:
    Marker0Marker1Factor(const gtsam::Key &key_t_m0_c, const gtsam::Key &key_t_m0_m1,
                         gtsam::Point2 m1_corner_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Point3 m1_corner_f_marker,
                         std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                         fvlam::Logger &logger,
                         bool throwCheirality = false) :
      Base(model, key_t_m0_c, key_t_m0_m1),
      key_t_m0_c_{key_t_m0_c}, key_t_m0_m1_{key_t_m0_m1},
      m1_corner_f_image_{m1_corner_f_image}, m1_corner_f_marker_{m1_corner_f_marker},
      cal3ds2_{cal3ds2}, logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }


    gtsam::Vector evaluateError(const gtsam::Pose3 &t_m0_c,
                                const gtsam::Pose3 &t_m0_m1,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      gtsam::Matrix66 inverse_d_pose3_wrt_pose3;
      gtsam::Matrix66 compose_d_pose3_wrt_pose3;
      gtsam::Matrix66 combined_d_pose3_wrt_pose3;
      gtsam::Matrix26 project_d_point2_wrt_pose3;

      // Find the inverse
      auto t_m1_m0 = t_m0_m1.inverse(H2 ? gtsam::OptionalJacobian<6, 6>(inverse_d_pose3_wrt_pose3) : boost::none);

      // find the pose of the camera in marker 1's frame.
      auto t_m1_c = t_m1_m0.compose(
        t_m0_c, H2 ? gtsam::OptionalJacobian<6, 6>(compose_d_pose3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_m1_c, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          m1_corner_f_marker_,
          (H1 || H2) ? gtsam::OptionalJacobian<2, 6>(project_d_point2_wrt_pose3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = project_d_point2_wrt_pose3;
        }
        if (H2) {
          *H2 = project_d_point2_wrt_pose3 * compose_d_pose3_wrt_pose3 * inverse_d_pose3_wrt_pose3;
        }

        // Return the error.
        return point_f_image - m1_corner_f_image_;

      } catch (gtsam::CheiralityException &e) {
        if (H1) *H1 = gtsam::Matrix26::Zero();
        if (H2) *H2 = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": t_m1_c " << gtsam::DefaultKeyFormatter(key_t_m0_c_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_t_m0_m1_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_t_m0_c_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };

// ==============================================================================
// QuadMarker0Marker1Factor class
// ==============================================================================

  class QuadMarker0Marker1Factor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
    using This = QuadMarker0Marker1Factor;
    using shared_ptr = boost::shared_ptr<This>;

    gtsam::Key key_t_m0_c_;
    gtsam::Key key_t_m0_m1_;
    std::array<gtsam::Point2, 4> m0_corners_f_image_;
    std::array<gtsam::Point2, 4> m1_corners_f_image_;
    std::array<gtsam::Point3, 4> corners_f_marker_;
    std::optional<gtsam::Pose3> t_camera_imager_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    fvlam::Logger &logger_;
    std::string debug_str_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)


  public:
    QuadMarker0Marker1Factor(const gtsam::Key &key_t_m0_c, const gtsam::Key &key_t_m0_m1,
                             std::array<gtsam::Point2, 4> m0_corners_f_image,
                             std::array<gtsam::Point2, 4> m1_corners_f_image,
                             const gtsam::SharedNoiseModel &model,
                             std::array<gtsam::Point3, 4> corners_f_marker,
                             std::optional<gtsam::Pose3> t_camera_imager,
                             std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
                             fvlam::Logger &logger,
                             std::string debug_str = std::string{},
                             bool throwCheirality = false) :
      Base(model, key_t_m0_c, key_t_m0_m1),
      key_t_m0_c_{key_t_m0_c}, key_t_m0_m1_{key_t_m0_m1},
      m0_corners_f_image_{std::move(m0_corners_f_image)},
      m1_corners_f_image_{std::move(m1_corners_f_image)},
      corners_f_marker_{std::move(corners_f_marker)},
      t_camera_imager_{t_camera_imager},
      cal3ds2_{cal3ds2}, logger_{logger},
      debug_str_{std::move(debug_str)},
      throwCheirality_{throwCheirality}
    {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }

    gtsam::Vector evaluateError(const gtsam::Pose3 &t_m0_c,
                                const gtsam::Pose3 &t_m0_m1,
                                boost::optional<gtsam::Matrix &> H1 = boost::none,
                                boost::optional<gtsam::Matrix &> H2 = boost::none) const override
    {
      gtsam::Matrix66 imager_d_pose3_wrt_pose3;
      gtsam::Matrix66 inverse_d_pose3_wrt_pose3;
      gtsam::Matrix66 compose_d_pose3_wrt_pose3;

      // Find the pose of the imager. If t_camera_imager is not optional, then the imager pose is
      // offset from the camera. Otherwise it is the same as the camera.
      auto t_m0_i = (t_camera_imager_) ? t_m0_c.compose(
        (*t_camera_imager_), (H1 || H2) ? gtsam::OptionalJacobian(imager_d_pose3_wrt_pose3) : boost::none) : t_m0_c;

      // Find the inverse of the transform from M1 to m0. This is used later to transform the imager
      // pose into m1's frame.
      auto t_m1_m0 = t_m0_m1.inverse(H2 ? gtsam::OptionalJacobian(inverse_d_pose3_wrt_pose3) : boost::none);

      // find the pose of the imager in marker 1's frame.
      auto t_m1_i = t_m1_m0.compose(
        t_m0_i, H2 ? gtsam::OptionalJacobian(compose_d_pose3_wrt_pose3) : boost::none);

      // Create two Pinhole Cameras - one in m0's frame and the other in m1's frame..
      auto m0_imager = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_m0_i, *cal3ds2_};
      auto m1_imager = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_m1_i, *cal3ds2_};

      try {
        bool get_H = H1 || H2;
        gtsam::Matrix26 J0, J1, J2, J3, J4, J5, J6, J7;
        auto e = (Eigen::Matrix<double, 16, 1>{}
          <<
          m0_imager.project(corners_f_marker_[0],
                            (get_H) ? gtsam::OptionalJacobian(J0) : boost::none) - m0_corners_f_image_[0],
          m0_imager.project(corners_f_marker_[1],
                            (get_H) ? gtsam::OptionalJacobian(J1) : boost::none) - m0_corners_f_image_[1],
          m0_imager.project(corners_f_marker_[2],
                            (get_H) ? gtsam::OptionalJacobian(J2) : boost::none) - m0_corners_f_image_[2],
          m0_imager.project(corners_f_marker_[3],
                            (get_H) ? gtsam::OptionalJacobian(J3) : boost::none) - m0_corners_f_image_[3],
          m1_imager.project(corners_f_marker_[0],
                            (get_H) ? gtsam::OptionalJacobian(J4) : boost::none) - m1_corners_f_image_[0],
          m1_imager.project(corners_f_marker_[1],
                            (get_H) ? gtsam::OptionalJacobian(J5) : boost::none) - m1_corners_f_image_[1],
          m1_imager.project(corners_f_marker_[2],
                            (get_H) ? gtsam::OptionalJacobian(J6) : boost::none) - m1_corners_f_image_[2],
          m1_imager.project(corners_f_marker_[3],
                            (get_H) ? gtsam::OptionalJacobian(J7) : boost::none) - m1_corners_f_image_[3]
        ).finished();

        if (t_camera_imager_) {
          J0 = J0 * imager_d_pose3_wrt_pose3;
          J1 = J1 * imager_d_pose3_wrt_pose3;
          J2 = J2 * imager_d_pose3_wrt_pose3;
          J3 = J3 * imager_d_pose3_wrt_pose3;
          J4 = J4 * imager_d_pose3_wrt_pose3;
          J5 = J5 * imager_d_pose3_wrt_pose3;
          J6 = J6 * imager_d_pose3_wrt_pose3;
          J7 = J7 * imager_d_pose3_wrt_pose3;
        }

        if (H1) {
          *H1 = (Eigen::Matrix<double, 16, 6>{}
            <<
            J0,
            J1,
            J2,
            J3,
            J4,
            J5,
            J6,
            J7
          ).finished();
        }

        if (H2) {
          gtsam::Matrix66 combined_d_pose3_wrt_pose3 = compose_d_pose3_wrt_pose3 * inverse_d_pose3_wrt_pose3;
          *H1 = (Eigen::Matrix<double, 16, 6>{}
            <<
            J0 * combined_d_pose3_wrt_pose3,
            J1 * combined_d_pose3_wrt_pose3,
            J2 * combined_d_pose3_wrt_pose3,
            J3 * combined_d_pose3_wrt_pose3,
            J4 * combined_d_pose3_wrt_pose3,
            J5 * combined_d_pose3_wrt_pose3,
            J6 * combined_d_pose3_wrt_pose3,
            J7 * combined_d_pose3_wrt_pose3
          ).finished();
        }

        return e;

      } catch (gtsam::CheiralityException &e) {
        if (H1) *H1 = Eigen::Matrix<double, 16, 6>::Zero();
        if (H2) *H2 = Eigen::Matrix<double, 16, 6>::Zero();

        logger_.error() << e.what() << ": " << debug_str_ << gtsam::DefaultKeyFormatter(key_t_m0_c_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_t_m0_m1_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_t_m0_c_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()}.replicate<8, 1>();
    }
  };

// ==============================================================================
// CalInfo class
// ==============================================================================

  struct CalInfo
  {
    using Map = std::map<std::string, CalInfo>;

    boost::shared_ptr<gtsam::Cal3DS2> cal3ds2_;
    std::shared_ptr<const gtsam::Cal3DS2> std_cal3ds2_;
    const fvlam::CameraInfo &camera_info_;
    const std::size_t imager_index_;

    CalInfo(boost::shared_ptr<gtsam::Cal3DS2> cal3ds2,
            const fvlam::CameraInfo &camera_info,
            std::size_t imager_index) :
      cal3ds2_{cal3ds2}, std_cal3ds2_{std::make_shared<const gtsam::Cal3DS2>(*cal3ds2)},
      camera_info_{camera_info}, imager_index_{imager_index}
    {}

    static Map MakeMap(const fvlam::MarkerModelRunner &runner)
    {
      auto map = Map{};
      std::size_t imager_index = 0;
      for (auto &cip : runner.model().camera_info_map().m()) {
        auto cal3ds2 = boost::shared_ptr<gtsam::Cal3DS2>(new gtsam::Cal3DS2(cip.second.to<gtsam::Cal3DS2>()));
        map.emplace(cip.first, CalInfo{cal3ds2, cip.second, imager_index++});
      }
      return map;
    }

    static std::vector<fvlam::Transform3> make_t_imager0_imagerNs(const Map &k_map)
    {
      std::vector<fvlam::Transform3> t_imager0_imagerNs{k_map.size()};
      for (auto &kp : k_map) {
        t_imager0_imagerNs[kp.second.imager_index_] = kp.second.camera_info_.t_camera_imager();
      }
      if (t_imager0_imagerNs.size() < 2 || !t_imager0_imagerNs[0].is_valid()) {
        return std::vector<fvlam::Transform3>{};
      }
      for (std::size_t i = 1; i < t_imager0_imagerNs.size(); i += 1) {
        t_imager0_imagerNs[i] = t_imager0_imagerNs[0].inverse() * t_imager0_imagerNs[i];
      }
      return t_imager0_imagerNs;
    }

    static std::string make_base_imager_frame_id(const Map &k_map)
    {
      // Find the CalInfo with index zero
      std::string base_imager_frame_id{};
      for (auto &cal_info : k_map) {
        if (cal_info.second.imager_index_ == 0) {
          base_imager_frame_id = cal_info.second.camera_info_.imager_frame_id();
          break;
        }
      }
      return base_imager_frame_id;
    }
  };

}
