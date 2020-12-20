
#include <iostream>
#include "catch2/catch.hpp"

#include "opencv2/core.hpp"

#include "gtsam/base/Vector.h"
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "../sho/sho_run.hpp"
#include "../../src/model.hpp"
#include "../../include/fvlam/camera_info.hpp"
#include "../../include/fvlam/marker_map.hpp"
#include "../../include/fvlam/marker_observation.hpp"
#include "../../include/fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  template<>
  Marker Marker::from<camsim::MarkerModel>(const camsim::MarkerModel &other)
  {
    auto f_tf = Transform3::from<gtsam::Pose3>(other.marker_f_world_);
    return Marker{other.key_, fvlam::Transform3WithCovariance{f_tf}};

  }
}

namespace camsim
{
#if 0

  TEST_CASE("sho_test - shonan_rotation_averaging")
  {
    REQUIRE(shonan_rotation_averaging() == 0);
  }

  TEST_CASE("sho_test - shonan_RA_simple")
  {
    REQUIRE(shonan_RA_simple() == 0);
  }

  TEST_CASE("sho_test - test_rotate3()")
  {
    REQUIRE(test_rotate3() == 0);
  }

  TEST_CASE("sho_test - inter_marker_rotation()")
  {
    REQUIRE(inter_marker_rotation() == 0);
  }

  const double degree = M_PI / 180;

  TEST_CASE("sho_test - Test Exp/Log")
  {
    int ret = 0;
//    std::vector<double> test_angles = {-89.999, -60, -45, -30, 0, 30, 45, 60, 89.999};
    std::vector<double> test_angles = {-400, -200, -100, -45, 0, 45, 100, 200, 400};
    std::vector<double> test_translate = {-1.0, 0, 1.0};

    for (auto &a : test_angles)
      a *= degree;
    for (auto rx : test_angles)
      for (auto ry : test_angles)
        for (auto rz : test_angles) {

          fvlam::Rotate3 SO3(fvlam::Rotate3::RzRyRx(rx, ry, rz));
          auto so3_fvlam = fvlam::Rotate3::Logmap(SO3);
          auto so3_gtsam = gtsam::Rot3::Logmap(gtsam::Rot3(SO3.rotation_matrix()));
          REQUIRE(gtsam::assert_equal(so3_gtsam(0), so3_fvlam(0), 1.0e-6));
          REQUIRE(gtsam::assert_equal(so3_gtsam(1), so3_fvlam(1), 1.0e-6));
          REQUIRE(gtsam::assert_equal(so3_gtsam(2), so3_fvlam(2), 1.0e-6));

//          std::cout << "so3 Logmap vs xyz "
//                    << so3_fvlam(0) / degree << "(" << x / degree << ") "
//                    << so3_fvlam(1) / degree << "(" << y / degree << ") "
//                    << so3_fvlam(2) / degree << "(" << z / degree << ") "
//                    << std::endl;

          auto SO3_fvlam = fvlam::Rotate3::Expmap(so3_fvlam);
          auto SO3_gtsam = gtsam::Rot3::Expmap(so3_gtsam);
          auto xyz_fvlam = SO3_fvlam.xyz();
          auto xyz_gtsam = SO3_gtsam.xyz();
          std::cout << "SO3 fvlam, gtsam(xyz) "
                    << xyz_fvlam(0) / degree << " " << xyz_gtsam(0) / degree << "(" << rx / degree << ") "
                    << xyz_fvlam(1) / degree << " " << xyz_gtsam(1) / degree << "(" << ry / degree << ") "
                    << xyz_fvlam(2) / degree << " " << xyz_gtsam(2) / degree << "(" << rz / degree << ") "
                    << std::endl;
          REQUIRE(gtsam::assert_equal(SO3_gtsam.matrix(), SO3_fvlam.rotation_matrix(), 1.0e-6));

          for (auto x : test_translate)
            for (auto y : test_translate)
              for (auto z : test_translate) {
                fvlam::Transform3 SE3(SO3, fvlam::Translate3(x, y, z));
                auto se3_fvlam = fvlam::Transform3::Logmap(SE3);
                auto se3_gtsam = gtsam::Pose3::Logmap(gtsam::Pose3(gtsam::Rot3(SO3.rotation_matrix()),
                                                                   gtsam::Vector3(x, y, z)));
                REQUIRE(gtsam::assert_equal(se3_gtsam(0), se3_fvlam(0), 1.0e-6));
                REQUIRE(gtsam::assert_equal(se3_gtsam(1), se3_fvlam(1), 1.0e-6));
                REQUIRE(gtsam::assert_equal(se3_gtsam(2), se3_fvlam(2), 1.0e-6));
                REQUIRE(gtsam::assert_equal(se3_gtsam(3), se3_fvlam(3), 1.0e-6));
                REQUIRE(gtsam::assert_equal(se3_gtsam(4), se3_fvlam(4), 1.0e-6));
                REQUIRE(gtsam::assert_equal(se3_gtsam(5), se3_fvlam(5), 1.0e-6));

                auto SE3_fvlam = fvlam::Transform3::Expmap(se3_fvlam);
                auto SE3_gtsam = gtsam::Pose3::Expmap(se3_gtsam);
                REQUIRE(gtsam::assert_equal(SE3_gtsam.rotation().matrix(), SE3_fvlam.r().rotation_matrix(), 1.0e-6));
                REQUIRE(gtsam::assert_equal(SE3_gtsam.translation(), SE3_fvlam.t().t(), 1.0e-6));
              }
        }
  }

#endif


  using CvCameraCalibration = std::pair<cv::Matx33d, cv::Vec<double, 5>>;

  struct TestParams
  {
    int n_markers = 16;
    int n_cameras = 64;

    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 1.0;
    double u_noise_sigma = 1.0;
  };

  static TestParams test_params;

  TEST_CASE("sho_test - Test Project function")
  {
#if 1
    ModelConfig model_config{camsim::MarkersConfigurations::tetrahedron,
                             camsim::CamerasConfigurations::z2_facing_origin,
                             camsim::CameraTypes::distorted_camera};

//    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{test_params.n_markers, 2.},
//                             PoseGens::SpinAboutZAtOriginFacingOut{test_params.n_cameras},
//                             camsim::CameraTypes::simulation,
//                             0.1775};
#else

    ModelConfig model_config{[]() -> std::vector<gtsam::Pose3>
                             {
                               return std::vector<gtsam::Pose3>{gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., 0.),
                                                                             gtsam::Point3{0., 0., -2.}}};
                             },
                             []() -> std::vector<gtsam::Pose3>
                             {
                               return std::vector<gtsam::Pose3>{gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                                                             gtsam::Point3{0., 0., 0.}}};
                             },
                             camsim::CameraTypes::simple_camera,
                             0.1775};
#endif

    Model model{model_config};
    auto marker_length = model.markers_.cfg_.marker_size_;

    auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(model.cameras_.calibration_);
    auto cv_camera_calibration = camera_calibration.to<CvCameraCalibration>();

    for (auto &m_camera : model.cameras_.cameras_) {
      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        cv_camera_calibration,
        fvlam::Transform3::from(m_camera.camera_f_world_),
        marker_length);

      for (auto &m_marker : model.markers_.markers_) {
        auto &corners_f_image = model.corners_f_images_[m_camera.index()][m_marker.index()];
        if (!corners_f_image.corners_f_image_.empty()) {
          auto f_marker = fvlam::Marker::from<MarkerModel>(m_marker);
          std::cout << "m_marker " << f_marker.to_id_string() << " "
                    << f_marker.to_corners_f_world_string(marker_length) << std::endl;

          auto observation = project_t_world_marker_function(f_marker);
          for (int i = 0; i < fvlam::MarkerObservation::Derived::MaxColsAtCompileTime; i += 1) {
            gtsam::Vector2 v0 = corners_f_image.corners_f_image_[i];
            gtsam::Vector2 v1 = observation.corners_f_image().col(i);
            REQUIRE(gtsam::assert_equal(v0, v1));
          }
        }
      }
    }
  }
}

