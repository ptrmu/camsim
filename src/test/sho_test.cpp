
#include <iostream>
#include "catch2/catch.hpp"

#include "opencv2/core.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "../sho/sho_run.hpp"
#include "../../include/fvlam/build_marker_map_interface.hpp"
#include "../../include/fvlam/camera_info.hpp"
#include "../../include/fvlam/marker_map.hpp"
#include "../../include/fvlam/marker_observation.hpp"
#include "../../include/fvlam/transform3_with_covariance.hpp"
#include "../../src/build_marker_map_runner.hpp"
#include "../../src/model.hpp"

namespace fvlam
{

  template<>
  Transform3 Transform3::from<camsim::CameraModel>(const camsim::CameraModel &other)
  {
    return Transform3::from<gtsam::Pose3>(other.camera_f_world_);
  }

  template<>
  Transform3 Transform3::from<camsim::MarkerModel>(const camsim::MarkerModel &other)
  {
    return Transform3::from<gtsam::Pose3>(other.marker_f_world_);
  }

  template<>
  Marker Marker::from<camsim::MarkerModel>(const camsim::MarkerModel &other)
  {
    return Marker{other.key_, fvlam::Transform3WithCovariance{Transform3::from(other)}};
  }
}

namespace camsim
{

  const double degree = (M_PI / 180.0);


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

  static auto pose_generator(const std::vector<fvlam::Transform3> &poses)
  {
    std::vector<gtsam::Pose3> ps;
    for (auto &pose : poses)
      ps.emplace_back(pose.to<gtsam::Pose3>());
    return [ps]()
    { return ps; };
  }

  static auto master_marker_length = 0.1775;
  static gtsam::Cal3DS2 master_cal3DS2{475, 475, 0, 400, 300, 0., 0.};
//  static gtsam::Cal3DS2 master_cal3DS2{1, 1, 0, 400, 300, 0., 0.};

  // Assuming world coordinate system is ENU
  // Camera coordinate system is Left,down,forward (along camera axis)
  static auto master_marker_pose_list = std::vector<fvlam::Transform3>{
    fvlam::Transform3{0, 0, 0, 0, 0, 0},
    fvlam::Transform3{0, 0, 0, 1, 0, 0},
    fvlam::Transform3{0, 0, 0, 1, 1, 0},
    fvlam::Transform3{0, 0, 0, 0, 1, 0},

    fvlam::Transform3{1 * degree, 0, 0, 0, 0, 0},
    fvlam::Transform3{1 * degree, 0, 0, 1, 1, 0},
  };

  static auto master_camera_pose_list = std::vector<fvlam::Transform3>{
    fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{180 * degree, 0, 0, 1, 0, 2},
    fvlam::Transform3{180 * degree, 0, 0, 1, 1, 2},
    fvlam::Transform3{180 * degree, 0, 0, 0, 1, 2},
    fvlam::Transform3{180 * degree, 0, 0, 0.01, 0, 2},

    fvlam::Transform3{181 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{181 * degree, 0, 0, 1, 1, 2},
    fvlam::Transform3{179 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{179 * degree, 0, 0, 1, 1, 2},

    fvlam::Transform3{181 * degree, 1 * degree, 1 * degree, 0, 0, 2},
    fvlam::Transform3{181 * degree, 1 * degree, 1 * degree, 1, 1, 2},
    fvlam::Transform3{179 * degree, -1 * degree, -1 * degree, 0, 0, 2},
    fvlam::Transform3{179 * degree, -1 * degree, -1 * degree, 1, 1, 2},
  };

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


  TEST_CASE("sho_test - Test Project function")
  {
#if 0
    ModelConfig model_config{camsim::MarkersConfigurations::tetrahedron,
                             camsim::CamerasConfigurations::z2_facing_origin,
                             camsim::CameraTypes::distorted_camera};

//    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{test_params.n_markers, 2.},
//                             PoseGens::SpinAboutZAtOriginFacingOut{test_params.n_cameras},
//                             camsim::CameraTypes::simulation,
//                             0.1775};
#else
    // Assuming world coordinate system is ENU
    auto marker_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{0, 0, 0, 0, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 1, 0},
      fvlam::Transform3{0, 0, 0, 0, 1, 0},
    };
    auto camera_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2},
//      fvlam::Transform3{180 * degree, 0, 0, 1, 0, 2},
//      fvlam::Transform3{180 * degree, 0, 0, 1, 1, 2},
//      fvlam::Transform3{180 * degree, 0, 0, 0, 1, 2},
    };


    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775};
#endif

    Model model{model_config};
    auto marker_length = model.markers_.cfg_.marker_size_;

    auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(model.cameras_.calibration_);
    auto cv_camera_calibration = camera_calibration.to<CvCameraCalibration>();
    auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

//    // Run some manual tests
//    auto cv_project_t_world_marker_function_0 = fvlam::Marker::project_t_world_marker(
//      cv_camera_calibration,
//      camera_pose_list[2],
//      marker_length);
//
//    auto gtsam_project_t_world_marker_function_0 = fvlam::Marker::project_t_world_marker(
//      gtsam_camera_calibration,
//      camera_pose_list[2],
//      marker_length);
//
//    auto f_marker_0 = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose_list[0]}};
//    auto cv_corners_f_image = cv_project_t_world_marker_function_0(f_marker_0);
//    auto gtsam_corners_f_image = gtsam_project_t_world_marker_function_0(f_marker_0);

    for (auto &m_camera : model.cameras_.cameras_) {
      auto cv_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        cv_camera_calibration,
        fvlam::Transform3::from(m_camera.camera_f_world_),
        marker_length);

      auto gtsam_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        fvlam::Transform3::from(m_camera.camera_f_world_),
        marker_length);

      for (auto &m_marker : model.markers_.markers_) {
        auto &corners_f_image = model.corners_f_images_[m_camera.index()][m_marker.index()];
        if (!corners_f_image.corners_f_image_.empty()) {
          auto f_marker = fvlam::Marker::from<MarkerModel>(m_marker);
          std::cout << "m_camera " << m_camera.key_
                    << "  m_marker " << m_marker.key_ << std::endl;

          // Test the OpenCV project_t_world_marker function
          auto cv_observation = cv_project_t_world_marker_function(f_marker);
          std::cout << "  cv     " << cv_observation.to_string() << std::endl;
          for (int i = 0; i < fvlam::MarkerObservation::ArraySize; i += 1) {
            gtsam::Vector2 v0 = corners_f_image.corners_f_image_[i];
            gtsam::Vector2 v1 = cv_observation.corners_f_image()[i].to<gtsam::Vector2>();
//            std::cout << "cv  " << fvlam::Translate2{v0}.to_string()
//                      << "  " << fvlam::Translate2{v1}.to_string() << std::endl;
//            REQUIRE(gtsam::assert_equal(v0, v1, 1.0e-4));
          }

          // Test the gtsam project_t_world_marker function
          auto gtsam_observation = gtsam_project_t_world_marker_function(f_marker);
          std::cout << "  gtsam  " << gtsam_observation.to_string() << std::endl;
          for (int i = 0; i < fvlam::MarkerObservation::ArraySize; i += 1) {
            gtsam::Vector2 v0 = corners_f_image.corners_f_image_[i];
            gtsam::Vector2 v1 = gtsam_observation.corners_f_image()[i].to<gtsam::Vector2>();
//            std::cout << "gt  " << fvlam::Translate2{v0}.to_string()
//                      << "  " << fvlam::Translate2{v1}.to_string() << std::endl;
//            REQUIRE(gtsam::assert_equal(v0, v1));
          }
        }
      }
    }
  }

  TEST_CASE("sho_test - Individual project test")
  {
    // Assuming world coordinate system is ENU
    auto marker_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{0, 0, 0, 0, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 1, 0},
      fvlam::Transform3{0, 0, 0, 0, 1, 0},
    };

    auto camera_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 1, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 1, 1, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 1, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0.01, 0, 2},
    };

    auto marker_length = 0.1775;
    gtsam::Cal3DS2 cal3DS2{475, 475, 0, 400, 300, 0., 0.};
    auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(cal3DS2);
    auto cv_camera_calibration = camera_calibration.to<CvCameraCalibration>();
    auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

    // Run some manual tests
    auto do_test = [
      &cv_camera_calibration,
      &gtsam_camera_calibration,
      marker_length](fvlam::Transform3 &camera_pose, fvlam::Transform3 &marker_pose)
    {
      auto cv_project_t_world_marker_function_0 = fvlam::Marker::project_t_world_marker(
        cv_camera_calibration,
        camera_pose,
        marker_length);

      auto gtsam_project_t_world_marker_function_0 = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        camera_pose,
        marker_length);

      auto f_marker_0 = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose}};
      auto cv_corners_f_image = cv_project_t_world_marker_function_0(f_marker_0);
      auto gtsam_corners_f_image = gtsam_project_t_world_marker_function_0(f_marker_0);

      std::cout << "camera pose " << camera_pose.to_string()
                << " marker pose " << marker_pose.to_string() << std::endl;
      std::cout << "cv    " << cv_corners_f_image.to_string() << std::endl
                << "gtsam " << gtsam_corners_f_image.to_string() << std::endl;
    };

//    do_test(camera_pose_list[0], marker_pose_list[0]);
    do_test(camera_pose_list[4], marker_pose_list[0]);
  }

  TEST_CASE("sho_test - Solve cv_solve_t_world_marker test")
  {
    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto do_test = [
      &cal3ds2 = model.cameras_.calibration_]
      (const fvlam::Transform3 &camera_pose,
       const fvlam::Transform3 &marker_pose)
    {
      auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(cal3ds2);
      auto cv_camera_calibration = camera_calibration.to<CvCameraCalibration>();
      auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

      auto cv_solve_t_world_marker_function = fvlam::Marker::solve_t_world_marker(
        cv_camera_calibration,
        camera_pose,
        master_marker_length);

      auto gtsam_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        camera_pose,
        master_marker_length);

//      std::cout << "camera pose  " << camera_pose.to_string()
//                << "    marker pose  " << marker_pose.to_string() << std::endl;

      auto f_marker = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose}};
      auto gtsam_corners_f_image = gtsam_project_t_world_marker_function(f_marker);
      auto cv_t_world_marker = cv_solve_t_world_marker_function(gtsam_corners_f_image);

//      std::cout << "     marker pose " << marker_pose.to_string() << std::endl
//                << "     calc pose   " << cv_t_world_marker.t_world_marker().tf().to_string() << std::endl;

      REQUIRE(gtsam::assert_equal(f_marker.t_world_marker().tf().mu(),
                                  cv_t_world_marker.t_world_marker().tf().mu(),
                                  1.0e-6));
    };

    for (auto &m_camera : model.cameras_.cameras_)
      for (auto &m_marker : model.markers_.markers_)
        do_test(fvlam::Transform3::from(m_camera), fvlam::Transform3::from(m_marker));
  }

  TEST_CASE("sho_test - Solve cv_solve_t_marker0_marker1 test")
  {
    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto do_test = [
      &cal3ds2 = model.cameras_.calibration_]
      (const fvlam::Transform3 &camera_f_world,
       const fvlam::Transform3 &marker0_f_world,
       const fvlam::Transform3 &marker1_f_world)
    {
//      std::cout << "camera pose  " << camera_f_world.to_string()
//                << "    marker0 pose  " << marker0_f_world.to_string()
//                << "    marker1 pose  " << marker1_f_world.to_string() << std::endl;

      auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(cal3ds2);
      auto cv_camera_calibration = camera_calibration.to<CvCameraCalibration>();
      auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        camera_f_world,
        master_marker_length);

      auto solve_t_marker0_marker1_function = fvlam::Marker::solve_t_marker0_marker1(
        cv_camera_calibration,
        master_marker_length);

      fvlam::Marker marker0{0, fvlam::Transform3WithCovariance{marker0_f_world}};
      fvlam::Marker marker1{0, fvlam::Transform3WithCovariance{marker1_f_world}};

      auto observation0 = project_t_world_marker_function(marker0);
      auto observation1 = project_t_world_marker_function(marker1);

      auto actual_t_marker0_marker1 = solve_t_marker0_marker1_function(observation0, observation1);

      auto expected_t_marker0_marker1 = marker0_f_world.inverse() * marker1_f_world;

//      std::cout << "     expected t_marker0_marker1 " << expected_t_marker0_marker1.to_string() << std::endl
//                << "     actual t_marker0_marker1   " << actual_t_marker0_marker1.tf().to_string() << std::endl;

      REQUIRE(gtsam::assert_equal(expected_t_marker0_marker1.mu(),
                                  actual_t_marker0_marker1.tf().mu(),
                                  1.0e-6));
    };

    for (auto m0 = 0; m0 < model.markers_.markers_.size(); m0 += 1)
      for (auto m1 = m0 + 1; m1 < model.markers_.markers_.size(); m1 += 1)
        for (auto &m_camera : model.cameras_.cameras_)
          do_test(fvlam::Transform3::from(m_camera),
                  fvlam::Transform3::from(model.markers_.markers_[m0]),
                  fvlam::Transform3::from(model.markers_.markers_[m1]));
  }
#endif

  TEST_CASE("sho_test - shonan build_marker_map from model")
  {
    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto map_initial = std::make_unique<fvlam::MarkerMap>(model.cfg_.marker_size_);
    auto marker_initial = fvlam::Marker{0, fvlam::Transform3WithCovariance{}, true};
    map_initial->add_marker(marker_initial);

    std::cout << "initial map\n" << map_initial->to_string() << std::endl;

    auto cxt = fvlam::BuildMarkerMapShonanContext(5);
    auto bmm_shonan = make_build_marker_map(cxt, std::move(map_initial));

    auto runner_config = BuildMarkerMapRunnerConfig{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(test_params.r_sigma),
        fvlam::Translate3::MuVector::Constant(test_params.t_sigma)).finished(),
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(test_params.r_sigma),
        fvlam::Translate3::MuVector::Constant(test_params.t_sigma)).finished(),
      fvlam::Translate2::MuVector::Constant(test_params.u_sampler_sigma),
      fvlam::Translate2::MuVector::Constant(test_params.u_noise_sigma),
      false
    };
    auto runner = BuildMarkerMapRunner(model, runner_config);

    auto map_solved = runner(*bmm_shonan);

    std::cout << "solved map\n" << map_solved->to_string() << std::endl;
  }
}

