
#include <iostream>
#include "catch2/catch.hpp"

#include "opencv2/core.hpp"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "../sho/sho_run.hpp"
#include "../../include/fvlam/build_marker_map_interface.hpp"
#include "../../include/fvlam/camera_info.hpp"
#include "../../include/fvlam/factors_gtsam.hpp"
#include "../../include/fvlam/localize_camera_interface.hpp"
#include "../../include/fvlam/logger.hpp"
#include "../../include/fvlam/marker.hpp"
#include "../../include/fvlam/model.hpp"
#include "../../include/fvlam/observation.hpp"
#include "../../include/fvlam/transform3_with_covariance.hpp"
#include "../../src/build_marker_map_runner.hpp"
#include "../../src/model.hpp"

namespace camsim
{
  const double degree = (M_PI / 180.0);

  struct TestParams
  {
    int n_markers = 16;
    int n_cameras = 64;

    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 0.0001;
    double u_noise_sigma = 0.01;

    fvlam::Logger::Levels logger_level = fvlam::Logger::Levels::level_debug;
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
    fvlam::Transform3{5 * degree, 5 * degree, 0, 1, 1, 0},
    fvlam::Transform3{0, 0, 5 * degree, 0, 1, 0},

    fvlam::Transform3{5 * degree, 0, 0, 0, 0, 0.25},
    fvlam::Transform3{-5 * degree, 0, 0, 1, 1, 0},
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

  TEST_CASE("sho_test - Test Exp/Log", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

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

//          logger.debug() << "so3 Logmap vs xyz "
//                    << so3_fvlam(0) / degree << "(" << x / degree << ") "
//                    << so3_fvlam(1) / degree << "(" << y / degree << ") "
//                    << so3_fvlam(2) / degree << "(" << z / degree << ") "
//                    << std::endl;

          auto SO3_fvlam = fvlam::Rotate3::Expmap(so3_fvlam);
          auto SO3_gtsam = gtsam::Rot3::Expmap(so3_gtsam);
          auto xyz_fvlam = SO3_fvlam.xyz();
          auto xyz_gtsam = SO3_gtsam.xyz();
          logger.debug() << "SO3 fvlam, gtsam(xyz) "
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

                auto SE3_fvlam_inverse = SE3_fvlam.inverse();
                auto SE3_gtsam_inverse = SE3_gtsam.inverse();
                REQUIRE(gtsam::assert_equal(SE3_gtsam_inverse.rotation().matrix(),
                                            SE3_fvlam_inverse.r().rotation_matrix(),
                                            1.0e-6));
                REQUIRE(gtsam::assert_equal(SE3_gtsam_inverse.translation(),
                                            SE3_fvlam_inverse.t().t(),
                                            1.0e-6));
              }
        }
  }

  TEST_CASE("sho_test - Test Project function", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

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
    auto marker_length = model.markers_.cfg_.marker_length_;

    auto camera_calibration = fvlam::CameraInfo::from(model.cameras_.calibration_);
    auto cv_camera_calibration = camera_calibration.to<fvlam::CvCameraCalibration>();
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
          auto f_marker = fvlam::Marker::from(m_marker);
          logger.debug() << "m_camera " << m_camera.key_
                         << "  m_marker " << m_marker.key_ << std::endl;

          // Test the OpenCV project_t_world_marker function
          auto cv_observation = cv_project_t_world_marker_function(f_marker);
          logger.debug() << "  cv     " << cv_observation.to_string() << std::endl;
          for (std::size_t i = 0; i < fvlam::Observation::ArraySize; i += 1) {
            gtsam::Vector2 v0 = corners_f_image.corners_f_image_[i];
            gtsam::Vector2 v1 = cv_observation.corners_f_image()[i].to<gtsam::Vector2>();
            logger.debug() << "cv  " << fvlam::Translate2{v0}.to_string()
                           << "  " << fvlam::Translate2{v1}.to_string() << std::endl;
//            REQUIRE(gtsam::assert_equal(v0, v1, 1.0e-4));
          }

          // Test the gtsam project_t_world_marker function
          auto gtsam_observation = gtsam_project_t_world_marker_function(f_marker);
          logger.debug() << "  gtsam  " << gtsam_observation.to_string() << std::endl;
          for (std::size_t i = 0; i < fvlam::Observation::ArraySize; i += 1) {
            gtsam::Vector2 v0 = corners_f_image.corners_f_image_[i];
            gtsam::Vector2 v1 = gtsam_observation.corners_f_image()[i].to<gtsam::Vector2>();
            logger.debug() << "gt  " << fvlam::Translate2{v0}.to_string()
                           << "  " << fvlam::Translate2{v1}.to_string() << std::endl;
            REQUIRE(gtsam::assert_equal(v0, v1));
          }
        }
      }
    }
  }

  TEST_CASE("sho_test - Individual project test", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    // Assuming world coordinate system is ENU
    auto marker_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{0, 0, 0, 0, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 1, 0},
      fvlam::Transform3{0, 0, 0, 0, 1, 0},
    };

    auto camera_pose_list = std::vector<fvlam::Transform3>{
      fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0.01, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0.02, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 0.01, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 0.02, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2.2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2.4},
      fvlam::Transform3{180 * degree, 0, 0, 1, 0, 2},
      fvlam::Transform3{180 * degree, 0, 0, 1, 1, 2},
      fvlam::Transform3{180 * degree, 0, 0, 0, 1, 2},
    };

    auto marker_length = 0.1775;
    gtsam::Cal3DS2 cal3DS2{475, 475, 0, 400, 300, 0., 0.};
    auto camera_calibration = fvlam::CameraInfo::from<gtsam::Cal3DS2>(cal3DS2);
    auto cv_camera_calibration = camera_calibration.to<fvlam::CvCameraCalibration>();
    auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

    // Run some manual tests
    auto do_test = [
      &cv_camera_calibration,
      &gtsam_camera_calibration,
      &logger,
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

      logger.debug() << "camera pose " << camera_pose.to_string()
                     << " marker pose " << marker_pose.to_string();
      logger.debug() << "cv    " << cv_corners_f_image.to_string();
      logger.debug() << "gtsam " << gtsam_corners_f_image.to_string();


      REQUIRE(cv_corners_f_image.equals(gtsam_corners_f_image, 1.0e-6));
    };

    //    do_test(camera_pose_list[0], marker_pose_list[0]);

    for (auto &camera_pose : camera_pose_list)
      for (auto &marker_pose : marker_pose_list)
        do_test(camera_pose, marker_pose);
  }

  class LocalizeCameraTest
  {
  public:
    struct Config
    {
      double marker_length = 2.0;

      bool use_cv_not_gtsam_ = true;

      double corner_measurement_sigma_ = 0.5;
      int gtsam_factor_type_ = 0;
      bool use_marker_covariance_ = false;

      double r_sampler_sigma_ = 0.0;
      double t_sampler_sigma_ = 0.0;
      double u_sampler_sigma_ = 0.001;

      double tolerance_ = 1.0e-2;

      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

  private:
    fvlam::Logger &logger_;
    const fvlam::MarkerModel &model_;
    const Config cfg_;

    const fvlam::Transform3::MuVector pose3_sampler_sigmas_;
    const fvlam::Translate2::MuVector point2_sampler_sigmas_;

    fvlam::MarkerMap map_;
    std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed_{};

    int frames_processed_{0};

    static fvlam::MarkerMap gen_map(
      const fvlam::MarkerModel &model)
    {
      auto map = fvlam::MarkerMap{model.environment()};
      for (auto &marker : model.targets()) {
        map.add_marker(marker);
      }
      return map;
    }

    static std::vector<fvlam::MarkerObservations> gen_marker_observations_list_perturbed(
      const fvlam::MarkerModel &model,
      const fvlam::Translate2::MuVector &point2_sampler_sigmas)
    {
      auto point2_sampler = gtsam::Sampler{point2_sampler_sigmas};

      std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed{};

      for (auto const &target_observations: model.target_observations_list()) {
        auto &observations_synced = target_observations.observations_synced();

        fvlam::ObservationsSynced perturbed_observations_synced{observations_synced.stamp(),
                                                                observations_synced.camera_frame_id()};
        for (auto &observations : observations_synced.v()) {

          fvlam::Observations perturbed_observations{observations.imager_frame_id()};
          for (auto &observation : observations.v()) {

            auto cfi = observation.corners_f_image();
            auto corners_f_image_perturbed = fvlam::Observation::Array{
              fvlam::Translate2{cfi[0].t() + point2_sampler.sample()},
              fvlam::Translate2{cfi[1].t() + point2_sampler.sample()},
              fvlam::Translate2{cfi[2].t() + point2_sampler.sample()},
              fvlam::Translate2{cfi[3].t() + point2_sampler.sample()},
            };
            fvlam::Observation perturbed_observation{observation.id(),
                                                     corners_f_image_perturbed,
                                                     observation.cov()};
            perturbed_observations.v_mutable().emplace_back(perturbed_observation);
          }
          perturbed_observations_synced.v_mutable().emplace_back(perturbed_observations);
        }
        fvlam::MarkerObservations perturbed_marker_observations{target_observations.camera_index(),
                                                                target_observations.t_map_camera(),
                                                                perturbed_observations_synced};
        marker_observations_list_perturbed.emplace_back(perturbed_marker_observations);
      }

      return marker_observations_list_perturbed;
    }

  public:
    using Maker = std::function<LocalizeCameraTest(fvlam::Logger &, fvlam::MarkerModel &)>;

    LocalizeCameraTest(fvlam::Logger &logger,
                       const fvlam::MarkerModel &model,
                       const Config &cfg) :
      logger_{logger},
      model_{model},
      cfg_{cfg},
      pose3_sampler_sigmas_{(fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(cfg.r_sampler_sigma_),
        fvlam::Translate3::MuVector::Constant(cfg.t_sampler_sigma_)).finished()},
      point2_sampler_sigmas_{fvlam::Translate2::MuVector::Constant(cfg.u_sampler_sigma_)},
      map_{gen_map(model_)},
      marker_observations_list_perturbed_{gen_marker_observations_list_perturbed(model_, point2_sampler_sigmas_)}
    {
      logger_.info() << "Model Markers:";
      for (auto &marker : model_.targets()) {
        logger_.info() << marker.to_string();
      }

      logger_.debug() << "Model Observations:";
      for (auto &to : model_.target_observations_list()) {
        logger_.debug() << "ObservationsSynced " << to.camera_index() << " "
                        << to.t_map_camera().to_string();
        for (auto &os : to.observations_synced().v()) {
          for (auto &o : os.v()) {
            auto &cs = o.corners_f_image();
            logger_.info() << os.imager_frame_id() << " "
                           << o.id() << " ("
                           << cs[0].t().transpose() << ") ("
                           << cs[1].t().transpose() << ") ( "
                           << cs[2].t().transpose() << ") ("
                           << cs[3].t().transpose() << ")";
          }
        }
      }
    }


    bool operator()(std::unique_ptr<fvlam::LocalizeCameraInterface> localize_camera)
    {
      frames_processed_ = 0;

      // Loop over the list of observations
      for (auto &marker_observation : marker_observations_list_perturbed_) {

        // Pass the perturbed observations to the localizer
        auto t_map_camera = localize_camera->solve_t_map_camera(marker_observation.observations_synced(),
                                                                model_.camera_info_map(), map_);

        if (!marker_observation.t_map_camera().equals(t_map_camera.tf(), cfg_.tolerance_)) {
          return false;
        }

        this->frames_processed_ += 1;
      }
      return true;
    }
  };

  TEST_CASE("sho_test - cv_solve_t_world_marker test", "[all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             master_marker_length,
                             true};

    Model model{model_config};

    auto do_test = [
      &logger,
      cal3ds2 = model.cameras_.calibration_,
      marker_length = model.cfg_.marker_length_]
      (const fvlam::Transform3 &camera_pose,
       const fvlam::Transform3 &marker_pose)
    {
      auto camera_calibration = fvlam::CameraInfo::from(cal3ds2);
      auto gtsam_camera_calibration = camera_calibration.to<gtsam::Cal3DS2>();

      auto gtsam_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        camera_pose,
        marker_length);

      double corner_measurement_sigma{1.0};
      bool use_marker_covariance{true};
      auto localize_camera_cv = make_localize_camera(fvlam::LocalizeCameraCvContext{}, logger);

      int gtsam_factor_resectioning{0};
      auto localize_camera_resectioning = make_localize_camera(
        fvlam::LocalizeCameraGtsamFactorContext{corner_measurement_sigma,
                                                gtsam_factor_resectioning,
                                                use_marker_covariance}, logger);

      int gtsam_factor_project_between{0};
      auto localize_camera_project_between = make_localize_camera(
        fvlam::LocalizeCameraGtsamFactorContext{corner_measurement_sigma,
                                                gtsam_factor_project_between,
                                                use_marker_covariance}, logger);

      int gtsam_factor_quad_resectioning{0};
      auto localize_camera_quad_resectioning = make_localize_camera(
        fvlam::LocalizeCameraGtsamFactorContext{corner_measurement_sigma,
                                                gtsam_factor_quad_resectioning,
                                                use_marker_covariance}, logger);

      logger.debug() << "camera pose  " << camera_pose.to_string()
                     << "    marker pose  " << marker_pose.to_string();

      // Generate observations from the marker and camera. If the projection results in invalid observations,
      // don't check.
      auto f_marker = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose}};
      auto gtsam_corners_f_image = gtsam_project_t_world_marker_function(f_marker);
      if (!gtsam_corners_f_image.is_valid()) {
        return;
      }
      logger.debug() << "             corners_f_image " << gtsam_corners_f_image.to_string();

      // Given observations and camera pose, solve to find the marker pose.
      auto cv_t_world_marker = camera_pose * gtsam_corners_f_image
        .solve_t_camera_marker(camera_calibration, master_marker_length);
      logger.debug() << "              cv marker pose   " << cv_t_world_marker.to_string();

      REQUIRE(gtsam::assert_equal(f_marker.t_map_marker().tf().mu(),
                                  cv_t_world_marker.mu(),
                                  1.0e-6));

      // Given observations and marker pose, solve to find the camera pose.
      fvlam::Observations observations{""};
      observations.v_mutable().emplace_back(gtsam_corners_f_image);
      fvlam::MarkerMap map{fvlam::MapEnvironment{"", 0, marker_length}};
      map.add_marker(f_marker);

      auto t_map_camera_cv = localize_camera_cv->solve_t_map_camera(observations, camera_calibration, map);
      logger.debug() << "              cv camera pose   " << t_map_camera_cv.tf().to_string();
      REQUIRE(camera_pose.equals(t_map_camera_cv.tf(), 1.0e-6));

      auto t_map_camera_resectioning = localize_camera_resectioning->solve_t_map_camera(observations,
                                                                                        camera_calibration, map);
      logger.debug() << "          resectioning camera pose   " << t_map_camera_resectioning.tf().to_string();
//      REQUIRE(camera_pose.equals(t_map_camera_resectioning.tf(), 1.0e-6));

      auto t_map_camera_project_between = localize_camera_project_between->solve_t_map_camera(observations,
                                                                                              camera_calibration, map);
      logger.debug() << "     camera_project camera pose   " << t_map_camera_project_between.tf().to_string();
      REQUIRE(camera_pose.equals(t_map_camera_project_between.tf(), 1.0e-6));

      auto t_map_camera_quad_resectioning = localize_camera_quad_resectioning->solve_t_map_camera(observations,
                                                                                                  camera_calibration,
                                                                                                  map);
      logger.debug() << "  quad resectioning camera pose   " << t_map_camera_quad_resectioning.tf().to_string();
      REQUIRE(camera_pose.equals(t_map_camera_quad_resectioning.tf(), 1.0e-6));
    };

    do_test(fvlam::Transform3::from(model.cameras_.cameras_[0]),
            fvlam::Transform3::from(model.markers_.markers_[1]));

    for (auto &m_camera : model.cameras_.cameras_)
      for (auto &m_marker : model.markers_.markers_)
        do_test(fvlam::Transform3::from(m_camera), fvlam::Transform3::from(m_marker));
  }

  TEST_CASE("sho_test - ResectioningFactor test", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto do_test = [
      &logger,
      &cal3ds2 = model.cameras_.calibration_,
      marker_length = model.cfg_.marker_length_]
      (const fvlam::Transform3 &camera_pose,
       const fvlam::Transform3 &marker_pose)
    {
      // Create a shared calibration object
      auto cal3ds2_ptr = std::make_shared<const gtsam::Cal3DS2>(cal3ds2);

      // Get corners_f_image
      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        cal3ds2, camera_pose, master_marker_length);
      auto f_marker = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose}};
      auto observation = project_t_world_marker_function(f_marker);
      if (!observation.is_valid()) {
        return;
      }

      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
      auto corners_f_world = f_marker.corners_f_world<std::vector<gtsam::Point3>>(marker_length);
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::vector<gtsam::Point3>>(marker_length);

      auto factor = fvlam::ResectioningFactor(corners_f_image[0], measurement_noise, 0,
                                              corners_f_world[0], cal3ds2_ptr, logger);

      gtsam::Matrix d_point2_wrt_camera;
      factor.evaluateError(camera_pose.to<gtsam::Pose3>(),
                           d_point2_wrt_camera);

      auto numericalH = gtsam::numericalDerivative11<gtsam::Point2, gtsam::Pose3>(
        [&factor](gtsam::Pose3 pose) -> gtsam::Point2
        {
          return factor.evaluateError(pose);
        }, camera_pose.to<gtsam::Pose3>());

      logger.debug() << "camera pose  " << camera_pose.to_string()
                     << "    marker pose  " << marker_pose.to_string();
      logger.debug() << "d_point2_wrt_camera\n" << d_point2_wrt_camera;
      logger.debug() << numericalH;

      REQUIRE(gtsam::assert_equal(numericalH, d_point2_wrt_camera, 1.0e-6));
    };

    for (auto &m_camera : model.cameras_.cameras_)
      for (auto &m_marker : model.markers_.markers_)
        do_test(fvlam::Transform3::from(m_camera), fvlam::Transform3::from(m_marker));
  }

  TEST_CASE("sho_test - ProjectBetweenFactor test", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto do_test = [
      &logger,
      &cal3ds2 = model.cameras_.calibration_,
      marker_length = model.cfg_.marker_length_]
      (const fvlam::Transform3 &camera_pose,
       const fvlam::Transform3 &marker_pose)
    {
      // Create a shared calibration object
      auto cal3ds2_ptr = std::make_shared<const gtsam::Cal3DS2>(cal3ds2);

      // Get corners_f_image
      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        cal3ds2, camera_pose, master_marker_length);
      auto f_marker = fvlam::Marker{0, fvlam::Transform3WithCovariance{marker_pose}};
      auto observation = project_t_world_marker_function(f_marker);
      if (!observation.is_valid()) {
        return;
      }
      auto corners_f_image = observation.corners_f_image();
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));

      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::vector<gtsam::Point3>>(marker_length);

      auto factor = fvlam::ProjectBetweenFactor(corners_f_image[0].to<gtsam::Point2>(), measurement_noise,
                                                0, 1,
                                                corners_f_marker[0], cal3ds2_ptr,
                                                logger);

      gtsam::Matrix d_point2_wrt_marker;
      gtsam::Matrix d_point2_wrt_camera;

      factor.evaluateError(marker_pose.to<gtsam::Pose3>(),
                           camera_pose.to<gtsam::Pose3>(),
                           d_point2_wrt_marker,
                           d_point2_wrt_camera);


      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
        {
          return factor.evaluateError(marker_pose, camera_pose);
        }, marker_pose.to<gtsam::Pose3>(), camera_pose.to<gtsam::Pose3>());
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
        {
          return factor.evaluateError(marker_pose, camera_pose);
        }, marker_pose.to<gtsam::Pose3>(), camera_pose.to<gtsam::Pose3>());

      logger.debug() << "camera pose  " << camera_pose.to_string()
                     << "    marker pose  " << marker_pose.to_string();
      logger.debug() << "d_point2_wrt_marker\n" << d_point2_wrt_marker;
      logger.debug() << numericalH1;
      logger.debug() << "d_point2_wrt_camera\n" << d_point2_wrt_camera;
      logger.debug() << numericalH2;

      REQUIRE(gtsam::assert_equal(numericalH1, d_point2_wrt_marker, 1.0e-6));
      REQUIRE(gtsam::assert_equal(numericalH2, d_point2_wrt_camera, 1.0e-6));
    };

//    do_test(fvlam::Transform3::from(model.cameras_.cameras_[0]),
//            fvlam::Transform3::from(model.markers_.markers_[1]));

    for (auto &m_camera : model.cameras_.cameras_)
      for (auto &m_marker : model.markers_.markers_)
        do_test(fvlam::Transform3::from(m_camera), fvlam::Transform3::from(m_marker));
  }

  TEST_CASE("sho_test - Test EstimateMeanAndCovariance", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    gtsam::Vector2 v0{1, 4};
    gtsam::Vector2 v1{3, 8};
    auto c0{gtsam::Vector2{0.01, 0.01}.asDiagonal()};
    auto c1{gtsam::Vector2{0.01, 0.01}.asDiagonal()};

    fvlam::EstimateMeanAndCovariance<gtsam::Vector2> emac{};
    emac.accumulate(v0);
    emac.accumulate(v1);

    auto mean = emac.mean();
    auto cov = emac.cov();

    logger.debug() << mean.transpose() << std::endl
                   << cov << std::endl;

    REQUIRE(gtsam::assert_equal(2., mean(0), 1.0e-4));
    REQUIRE(gtsam::assert_equal(6., mean(1), 1.0e-4));

    REQUIRE(gtsam::assert_equal(1., cov(0, 0), 1.0e-4));
    REQUIRE(gtsam::assert_equal(4., cov(1, 1), 1.0e-4));
    REQUIRE(gtsam::assert_equal(2., cov(0, 1), 1.0e-4));
  }

  TEST_CASE("sho_test - Test EstimateMeanAndCovariance with sampler", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    std::vector<fvlam::Translate3::MuVector> sigmas{
      fvlam::Translate3::MuVector{0.1, 0.0, 0.0},
      fvlam::Translate3::MuVector{0.1, 0.1, 0.0},
      fvlam::Translate3::MuVector{0.1, 0.1, 0.1},
    };

    for (auto &sigma : sigmas) {
      gtsam::Sampler sampler{sigma};
      fvlam::EstimateMeanAndCovarianceSimple<fvlam::Translate3::MuVector> emac{};
      fvlam::EstimateMeanAndCovariance2PSimple<fvlam::Translate3::MuVector> emac_2p{};

      for (int i = 0; i < 10000; i += 1) {
        fvlam::Translate3::MuVector sample{sampler.sample()};
        emac.accumulate(sample);
        emac_2p.accumulate(sample);
      }

      auto mean = emac.mean();
      auto cov = emac.cov();
      auto mean_2p = emac_2p.mean();
      auto cov_2p = emac_2p.cov();

      logger.debug() << "mu" << std::endl;
      fvlam::Translate3WithCovariance twc{fvlam::Translate3::from(mean), cov};
      logger.debug() << twc.to_string() << std::endl;
      fvlam::Translate3WithCovariance twc_2p{fvlam::Translate3::from(mean_2p), cov_2p};
      logger.debug() << twc_2p.to_string() << std::endl;

      REQUIRE(gtsam::assert_equal<fvlam::Translate3::MuVector>(fvlam::Translate3::MuVector::Zero(), mean, 1.0e-2));
      REQUIRE(gtsam::assert_equal<fvlam::Translate3::MuVector>(mean, mean_2p, 1.0e-6));
      REQUIRE(gtsam::assert_equal<double>(sigma[0] * sigma[0], cov(0, 0), 1.0e-3));
      REQUIRE(gtsam::assert_equal<double>(sigma[1] * sigma[0], cov(1, 1), 1.0e-3));
      REQUIRE(gtsam::assert_equal<double>(sigma[2] * sigma[0], cov(2, 2), 1.0e-3));
      REQUIRE(gtsam::assert_equal<fvlam::EstimateMeanAndCovarianceSimple
        <fvlam::Translate3::MuVector>::CovarianceMatrix>(cov, cov_2p, 1.0e-6));
    }
  }

  TEST_CASE("sho_test - Test EstimatePoseMeanAndCovariance with sampler", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    using Pose = fvlam::Transform3;
    using PoseMu = fvlam::Transform3::MuVector;

    std::vector<Pose> poses{
      Pose{1 * degree, 1 * degree, 1 * degree, 2, 1, 1},
      Pose{81 * degree, 1 * degree, 1 * degree, 1, 1, 2},
      Pose{79 * degree, -1 * degree, -1 * degree, 0, 0, 2},
      Pose{79 * degree, -1 * degree, -1 * degree, 1, 1, 2},
      Pose{181 * degree, 1 * degree, 1 * degree, 2, 1, 1},
      Pose{181 * degree, 1 * degree, 1 * degree, 1, 1, 2},
      Pose{179 * degree, -1 * degree, -1 * degree, 0, 0, 2},
      Pose{179 * degree, -1 * degree, -1 * degree, 1, 1, 2},
    };

    std::vector<PoseMu> sigmas{
      (PoseMu{} << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.0, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.1, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.0, 0.0, 0.2, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.0, 0.2, 0.2, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished(),
    };

    for (auto pose : poses)
      for (auto &sigma : sigmas) {
        gtsam::Sampler sampler{sigma};
        fvlam::EstimateTransform3MeanAndCovarianceOnManifold emac{};

        for (int i = 0; i < 10000; i += 1) {
          PoseMu offset = sampler.sample();
          auto sample{pose.retract(offset)};
//          Pose sample{pose.mu() + sampler.sample()};
          emac.accumulate(sample);
        }

        auto mean = emac.mean();
        auto cov = emac.cov();

        fvlam::Transform3WithCovariance twc{mean, cov};
        logger.debug() << twc.to_string() << std::endl;

        REQUIRE(gtsam::assert_equal<fvlam::Rotate3::CovarianceMatrix>(pose.r().rotation_matrix(),
                                                                      mean.r().rotation_matrix(), 1.0e-2));
        REQUIRE(gtsam::assert_equal<fvlam::Translate3::MuVector>(pose.t().t(), mean.t().t(), 1.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[0] * sigma[0], cov(0, 0), 5.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[1] * sigma[1], cov(1, 1), 5.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[2] * sigma[2], cov(2, 2), 5.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[3] * sigma[3], cov(3, 3), 5.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[4] * sigma[4], cov(4, 4), 5.0e-2));
        REQUIRE(gtsam::assert_equal<double>(sigma[5] * sigma[5], cov(5, 5), 5.0e-2));
      }
  }

  TEST_CASE("sho_test - Test covariance conversion", "[.][all]")
  {
    TestParams tp{};
    fvlam::LoggerCout logger(tp.logger_level);

    using Pose = fvlam::Transform3;
    using PoseMu = fvlam::Transform3::MuVector;

    std::vector<Pose> poses{
      Pose{1 * degree, 1 * degree, 1 * degree, 2, 1, 1},
      Pose{81 * degree, 1 * degree, 1 * degree, 1, 1, 2},
      Pose{79 * degree, -1 * degree, -1 * degree, 0, 0, 2},
      Pose{79 * degree, -1 * degree, -1 * degree, 1, 1, 2},
      Pose{181 * degree, 1 * degree, 1 * degree, 2, 1, 1},
      Pose{181 * degree, 1 * degree, 1 * degree, 1, 1, 2},
      Pose{179 * degree, -1 * degree, -1 * degree, 0, 0, 2},
      Pose{179 * degree, -1 * degree, -1 * degree, 1, 1, 2},
    };

    std::vector<PoseMu> sigmas{
      (PoseMu{} << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.0, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.1, 0.0, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.0, 0.0, 0.2, 0.0, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.0, 0.2, 0.2, 0.0).finished(),
      (PoseMu{} << 0.1, 0.1, 0.1, 0.2, 0.2, 0.2).finished(),
    };

    for (auto pose : poses)
      for (auto &sigma : sigmas) {

        auto pose_gtsam = pose.to<gtsam::Pose3>();
        auto cov = sigma * sigma.transpose();

        auto cov_gtsam = fvlam::GtsamUtil::cov_gtsam_from_ros(pose_gtsam, cov);
        auto cov_actual = fvlam::GtsamUtil::cov_ros_from_gtsam(pose_gtsam, cov_gtsam);

        REQUIRE(gtsam::assert_equal<fvlam::Transform3::CovarianceMatrix>(cov, cov_actual));
      }
  }

  TEST_CASE("sho_test - shonan build from model", "[.][all]")
  {
    struct TestParams
    {
      double r_sampler_sigma = 0.0;
      double t_sampler_sigma = 0.0;
      double u_sampler_sigma = 0.00001;
      double r_noise_sigma = 0.1;
      double t_noise_sigma = 0.1;
      double u_noise_sigma = 0.5;
    };

    TestParams tp;

    fvlam::LoggerCout logger{fvlam::Logger::level_info};

    ModelConfig model_config{pose_generator(master_marker_pose_list),
                             pose_generator(master_camera_pose_list),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto map_initial = std::make_unique<fvlam::MarkerMap>(fvlam::MapEnvironment{"", 0, model.cfg_.marker_length_});
    auto marker_initial = fvlam::Marker{0, fvlam::Transform3WithCovariance{}, true};
    map_initial->add_marker(marker_initial);

//    logger.debug() << "initial map\n" << map_initial->to_string() << std::endl;

    auto solve_tmm_context = fvlam::SolveTmmContextCvSolvePnp{false};
    auto solve_tmm_factory = fvlam::make_solve_tmm_factory(solve_tmm_context,
                                                           model.cfg_.marker_length_);

    auto tmm_context = fvlam::BuildMarkerMapTmmContext(solve_tmm_factory,
                                                       true,
                                                       fvlam::BuildMarkerMapTmmContext::NoiseStrategy::minimum,
                                                       0.1, 0.3);
    auto bmm_shonan = make_build_marker_map(tmm_context, logger, *map_initial);

    auto runner_config = BuildMarkerMapRunnerConfig{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(tp.r_sampler_sigma),
        fvlam::Translate3::MuVector::Constant(tp.t_sampler_sigma)).finished(),
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(tp.r_noise_sigma),
        fvlam::Translate3::MuVector::Constant(tp.t_noise_sigma)).finished(),
      fvlam::Translate2::MuVector::Constant(tp.u_sampler_sigma),
      fvlam::Translate2::MuVector::Constant(tp.u_noise_sigma),
      false
    };
    auto runner = BuildMarkerMapRunner(model, runner_config);

    auto map_solved = runner(*bmm_shonan);

    logger.debug() << "solved map\n" << map_solved->to_string() << std::endl;
  }
}

