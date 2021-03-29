
#include <iostream>
#include "catch2/catch.hpp"

#include "../../include/fvlam/build_marker_map_interface.hpp"
#include "../../include/fvlam/camera_info.hpp"
#include "../../include/fvlam/logger.hpp"
#include "../../include/fvlam/marker.hpp"
#include "../../include/fvlam/model.hpp"
#include "../../include/fvlam/observation.hpp"
#include "../../include/fvlam/transform3_with_covariance.hpp"
#include "../../src/build_marker_map_runner.hpp"
#include "../../src/model.hpp"

namespace camsim
{
// ==============================================================================
// BuildMarkerMapTest class
// ==============================================================================

  constexpr double degree = M_PI / 180.;

  class BuildMarkerMapTest
  {
  public:
    struct Config
    {
      int n_markers_ = 8;
      int n_cameras_ = 64;

      bool average_on_space_not_manifold_ = false;
      bool use_shonan_initial_ = false;
      fvlam::BuildMarkerMapTmmContext::NoiseStrategy noise_strategy_ =
        fvlam::BuildMarkerMapTmmContext::NoiseStrategy::minimum;

      double r_sampler_sigma_ = 0.0;
      double t_sampler_sigma_ = 0.0;
      double u_sampler_sigma_ = 0.001;
      double r_noise_sigma_ = 0.1;
      double t_noise_sigma_ = 0.3;
      double u_noise_sigma_ = 0.5;

      double tolerance_ = 1.0e-2;

      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

  private:
    fvlam::Logger &logger_;
    const fvlam::MarkerModel &model_;
    const Config cfg_;

    const fvlam::Transform3::MuVector pose3_sampler_sigmas_;
    const fvlam::Transform3::MuVector pose3_noise_sigmas_;
    const fvlam::Translate2::MuVector point2_sampler_sigmas_;
    const fvlam::Translate2::MuVector point2_noise_sigmas_;

    std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed_{};

    int frames_processed_{0};

  public:
    using Maker = std::function<BuildMarkerMapTest(fvlam::Logger &, fvlam::MarkerModel &)>;

    BuildMarkerMapTest(fvlam::Logger &logger,
                       const fvlam::MarkerModel &model,
                       const Config &cfg) :
      logger_{logger},
      model_{model},
      cfg_{cfg},
      pose3_sampler_sigmas_{(fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(cfg.r_sampler_sigma_),
        fvlam::Translate3::MuVector::Constant(cfg.t_sampler_sigma_)).finished()},
      pose3_noise_sigmas_{(fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(cfg.r_noise_sigma_),
        fvlam::Translate3::MuVector::Constant(cfg.t_noise_sigma_)).finished()},
      point2_sampler_sigmas_{fvlam::Translate2::MuVector::Constant(cfg.u_sampler_sigma_)},
      point2_noise_sigmas_{fvlam::Translate2::MuVector::Constant(cfg.u_noise_sigma_)}
    {
      auto point2_sampler = gtsam::Sampler{point2_sampler_sigmas_};

      for (auto const &target_observations: model_.target_observations_list()) {
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
        marker_observations_list_perturbed_.emplace_back(perturbed_marker_observations);
      }


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


    bool operator()(std::unique_ptr<fvlam::BuildMarkerMapInterface> build_marker_map)
    {
      frames_processed_ = 0;

      // Loop over the list of observations
      for (auto &marker_observation : marker_observations_list_perturbed_) {

        // Pass the perturbed observations to the builder
        build_marker_map->process(marker_observation.observations_synced(), model_.camera_info_map());

        this->frames_processed_ += 1;
      }

      // Build the map.
      auto built_map = build_marker_map->build();
      auto error = fvlam::BuildMarkerMapTmmContext::BuildError::from(*build_marker_map, *built_map);

      logger_.info() << "Resulting Markers:\n"
                     << built_map->to_string() << "\n"
                     << error.to_string();

      return check_maps(*built_map);
    }

  private:
    bool check_maps(const fvlam::MarkerMap &built_map)
    {
      int n{0};
      double r_error_sq_accum{0};
      double t_error_sq_accum{0};

      for (auto &model_marker : model_.targets()) {
        auto it = built_map.find_marker_const(model_marker.id());
        if (it == nullptr) {
          return false;
        }

        logger_.debug() << "  " << it->to_string();
        if (!model_marker.t_map_marker().tf().equals(it->t_map_marker().tf(), cfg_.tolerance_, true)) {
          return false;
        }

        n += 1;
        auto model_mu = model_marker.t_map_marker().tf().mu();
        auto solve_mu = it->t_map_marker().tf().mu();
        r_error_sq_accum += (model_mu.head<3>() - solve_mu.head<3>()).cwiseAbs().sum() / 3.;
        t_error_sq_accum += (model_mu.tail<3>() - solve_mu.tail<3>()).cwiseAbs().sum() / 3.;
      }

      double r_error = std::sqrt(r_error_sq_accum / n);
      double t_error = std::sqrt(t_error_sq_accum / n);
      logger_.info() << "True map error - r:" << r_error << " t:" << t_error;
      return true;
    }
  };

  static bool test_build_marker_map_tmm(
    BuildMarkerMapTest::Config bmm_test_config,
    fvlam::MarkerModel::Maker model_maker)
  {

    fvlam::LoggerCout logger{bmm_test_config.logger_level_};

    auto test_maker = [&bmm_test_config](fvlam::Logger &logger,
                                         fvlam::MarkerModel &model) -> BuildMarkerMapTest
    {
      return BuildMarkerMapTest(logger, model, bmm_test_config);
    };


    auto runner = fvlam::Runner<fvlam::MarkerModel, BuildMarkerMapTest,
      std::unique_ptr<fvlam::BuildMarkerMapInterface>>
      (logger, model_maker, test_maker);


    auto uut_maker = [&bmm_test_config](fvlam::Logger &logger,
                                        fvlam::MarkerModel &model) -> std::unique_ptr<fvlam::BuildMarkerMapInterface>
    {
      auto map_initial = std::make_unique<fvlam::MarkerMap>(model.environment());
      map_initial->add_marker(fvlam::Marker(model.targets()[0].id(), model.targets()[0].t_map_marker(), true));

      auto solve_tmm_context = fvlam::SolveTmmContextCvSolvePnp{bmm_test_config.average_on_space_not_manifold_};
      auto solve_tmm_factory = fvlam::make_solve_tmm_factory(solve_tmm_context,
                                                             model.environment().marker_length());

      auto tmm_context = fvlam::BuildMarkerMapTmmContext(solve_tmm_factory,
                                                         bmm_test_config.use_shonan_initial_,
                                                         bmm_test_config.noise_strategy_,
                                                         bmm_test_config.r_noise_sigma_,
                                                         bmm_test_config.t_noise_sigma_);
      return make_build_marker_map(tmm_context, logger, *map_initial);
    };

    return runner(uut_maker);
  }

  static void test_four_build_marker_map_tmm(
    BuildMarkerMapTest::Config bmm_test_config,
    fvlam::MarkerModel::Maker model_maker)
  {
    bmm_test_config.average_on_space_not_manifold_ = false;
    bmm_test_config.use_shonan_initial_ = false;
    REQUIRE(test_build_marker_map_tmm(bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = true;
    bmm_test_config.use_shonan_initial_ = false;
    REQUIRE(test_build_marker_map_tmm(bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = false;
    bmm_test_config.use_shonan_initial_ = true;
    REQUIRE(test_build_marker_map_tmm(bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = true;
    bmm_test_config.use_shonan_initial_ = true;
    REQUIRE(test_build_marker_map_tmm(bmm_test_config, model_maker));
  }

  TEST_CASE("build_marker_map_tmm - rotating camera - match markers", "[.][all]")
  {
    auto bmm_test_config = BuildMarkerMapTest::Config();

    auto model_maker = [&bmm_test_config]() -> fvlam::MarkerModel
    {
      fvlam::MarkerModel model(fvlam::MapEnvironmentGen::Default(),
                               fvlam::CameraInfoMapGen::Dual(),
                               fvlam::CamerasGen::SpinAboutZAtOriginFacingOut(bmm_test_config.n_cameras_),
                               fvlam::MarkersGen::CircleInXYPlaneFacingOrigin(bmm_test_config.n_markers_, 2));
      return model;
    };

    test_four_build_marker_map_tmm(bmm_test_config, model_maker);
  }

  TEST_CASE("build_marker_map_tmm - space accumulate - match markers", "[.][all]")
  {
    auto bmm_test_config = BuildMarkerMapTest::Config();

    auto model_maker = [&bmm_test_config]() -> fvlam::MarkerModel
    {
      fvlam::MarkerModel model(fvlam::MapEnvironmentGen::Default(),
                               fvlam::CameraInfoMapGen::DualWideAngle(),
                               fvlam::CamerasGen::LookingDownZ(2.0),
                               fvlam::MarkersGen::CircleInXYPlaneFacingAlongZ(
                                 8, 1.0, 0.0, true));
      return model;
    };

    test_four_build_marker_map_tmm(bmm_test_config, model_maker);
  }

  TEST_CASE("build_marker_map_tmm - build_marker_map_tmm circle of markers, camera in circle", "[.][all]")
  {
    auto bmm_test_config = BuildMarkerMapTest::Config();

    auto model_maker = [&bmm_test_config]() -> fvlam::MarkerModel
    {
      fvlam::MarkerModel model(fvlam::MapEnvironmentGen::Default(),
                               fvlam::CameraInfoMapGen::Simulation(),
                               fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                 8, 1.0, 2.0, false),
                               fvlam::MarkersGen::CircleInXYPlaneFacingAlongZ(
                                 8, 1.0, 0.0, true));
      return model;
    };

    test_four_build_marker_map_tmm(bmm_test_config, model_maker);
  }

  TEST_CASE("build_marker_map_tmm - build_marker_map_tmm from model", "[.][all]")
  {
    // Assuming world coordinate system is ENU
    // Marker coordinate system is also ENU
    static auto marker_pose_list_0 = std::vector<fvlam::Transform3>{
      fvlam::Transform3{0, 0, 0, 0, 0, 0},
      fvlam::Transform3{0, 0, 0, 1, 0, 0},
      fvlam::Transform3{5 * degree, 5 * degree, 0, 1, 1, 0},
      fvlam::Transform3{0, 0, 5 * degree, 0, 1, 0},

      fvlam::Transform3{5 * degree, 0, 0, 0, 0, 0.25},
      fvlam::Transform3{-5 * degree, 0, 0, 1, 1, 0},
    };

    // Assuming world coordinate system is ENU
    // Camera coordinate system is right,down,forward (along camera axis)
    static auto camera_pose_list_0 = std::vector<fvlam::Transform3>{
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

    auto bmm_test_config = BuildMarkerMapTest::Config();

    auto model_maker = [&bmm_test_config]() -> fvlam::MarkerModel
    {
      fvlam::MarkerModel model(fvlam::MapEnvironmentGen::Default(),
                               fvlam::CameraInfoMapGen::Simulation(),
                               camera_pose_list_0,
                               fvlam::MarkersGen::TargetsFromTransform3s(marker_pose_list_0));
      return model;
    };

    test_four_build_marker_map_tmm(bmm_test_config, model_maker);
  }
}
