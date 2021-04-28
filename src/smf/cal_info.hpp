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
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
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

  class Imager0Imager1Factor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
    using This = Imager0Imager1Factor;
    using shared_ptr = boost::shared_ptr<This>;

    gtsam::Key key_t_w_i0_;
    gtsam::Key key_t_i0_i1_;
    gtsam::Point2 point_f_image_;
    gtsam::Point3 point_f_marker_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    fvlam::Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)


  public:
    Imager0Imager1Factor(const gtsam::Key &key_t_w_i0, const gtsam::Key &key_t_i0_i1,
                         gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Point3 point_f_marker,
                         std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                         fvlam::Logger &logger,
                         bool throwCheirality = false) :
      Base(model, key_t_w_i0, key_t_i0_i1),
      key_t_w_i0_{key_t_w_i0}, key_t_i0_i1_{key_t_i0_i1},
      point_f_image_{point_f_image}, point_f_marker_{point_f_marker},
      cal3ds2_{cal3ds2}, logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }


    gtsam::Vector evaluateError(const gtsam::Pose3 &t_w_i0,
                                const gtsam::Pose3 &t_i0_i1,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
      gtsam::Matrix66 d_pose3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;

      // Transform the point from the Marker frame to the World frame
      auto t_w_i1 = t_w_i0.compose(
        t_i0_i1,
        H2 ? gtsam::OptionalJacobian<6, 6>(d_pose3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_w_i1, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_marker_,
          (H1 || H2) ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3 * d_pose3_wrt_pose3;
        }

        // Return the error.
        return point_f_image - point_f_image_;

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
  };

}