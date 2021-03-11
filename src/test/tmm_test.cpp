
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
  constexpr double degree = M_PI / 180.;

  struct TestParams
  {
    int n_markers = 8;
    int n_cameras = 64;

    double r_sampler_sigma = 0.0;
    double t_sampler_sigma = 0.0;
    double u_sampler_sigma = 0.00025;
    double r_noise_sigma = 0.1;
    double t_noise_sigma = 0.3;
    double u_noise_sigma = 0.5;

    double tolerance = 4.0e-1;

    fvlam::Logger::Levels logger_level = fvlam::Logger::Levels::level_info;
  };

  static auto create_pose_generator(const std::vector<fvlam::Transform3> &poses)
  {
    std::vector<gtsam::Pose3> ps;
    for (auto &pose : poses)
      ps.emplace_back(pose.to<gtsam::Pose3>());
    return [ps]()
    { return ps; };
  }

  struct MapAndError
  {
    std::unique_ptr<fvlam::MarkerMap> map_;
    fvlam::BuildMarkerMapTmmContext::BuildError error_;

    MapAndError(std::unique_ptr<fvlam::MarkerMap> map,
                fvlam::BuildMarkerMapTmmContext::BuildError error) :
      map_{std::move(map)}, error_{error}
    {}
  };

  static std::vector<MapAndError> run_solvers(Model &model, TestParams &tp, fvlam::LoggerCout &logger)
  {
    std::vector<MapAndError> solved_maps{};

    auto map_initial = std::make_unique<fvlam::MarkerMap>(fvlam::MapEnvironment{"", 0, model.cfg_.marker_length_});
    map_initial->add_marker(fvlam::Marker{
      0,
      fvlam::Transform3WithCovariance{fvlam::Transform3::from(model.markers_.markers_[0].marker_f_world_)},
      true});

    auto runner_config = BuildMarkerMapRunnerConfig{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(tp.r_sampler_sigma),
        fvlam::Translate3::MuVector::Constant(tp.t_sampler_sigma)).finished(),
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(tp.r_noise_sigma),
        fvlam::Translate3::MuVector::Constant(tp.t_noise_sigma)).finished(),
      fvlam::Translate2::MuVector::Constant(tp.u_sampler_sigma),
      fvlam::Translate2::MuVector::Constant(tp.u_noise_sigma),
      false
    };

    BuildMarkerMapRunner bmm_runner{model, runner_config};

    auto make_bmm_tmm = [
      &model,
      &tp,
      &logger,
      &map_initial,
      &bmm_runner](
      bool average_on_space_not_manifold,
      bool use_shonan_initial) -> MapAndError
    {
      auto solve_tmm_context = fvlam::SolveTmmContextCvSolvePnp{average_on_space_not_manifold};
      auto solve_tmm_factory = fvlam::make_solve_tmm_factory(solve_tmm_context,
                                                             model.cfg_.marker_length_);

      auto tmm_context = fvlam::BuildMarkerMapTmmContext(solve_tmm_factory,
                                                         use_shonan_initial,
                                                         fvlam::BuildMarkerMapTmmContext::NoiseStrategy::minimum,
                                                         tp.r_noise_sigma, tp.t_noise_sigma);
      auto map_builder = make_build_marker_map(tmm_context, logger, *map_initial);

      auto built_map = bmm_runner(*map_builder);
      auto error = fvlam::BuildMarkerMapTmmContext::BuildError::from(*map_builder, *built_map);
      return MapAndError{std::move(built_map), error};
    };

    solved_maps.emplace_back(make_bmm_tmm(false, true));
    solved_maps.emplace_back(make_bmm_tmm(true, true));
    solved_maps.emplace_back(make_bmm_tmm(false, false));
    solved_maps.emplace_back(make_bmm_tmm(true, false));

    return solved_maps;
  };

  static std::pair<double, double> calc_error(const Model &model, const fvlam::MarkerMap &solved_map)
  {
    int n{0};
    double r_error_sq_accum{0};
    double t_error_sq_accum{0};
    for (auto &m_marker : model.markers_.markers_) {
      auto it = solved_map.find_marker_const(m_marker.index());
      if (it != nullptr) {
        n += 1;
        auto model_mu = fvlam::Transform3::from(m_marker.marker_f_world_).mu();
        auto solve_mu = it->t_map_marker().tf().mu();
        r_error_sq_accum += (model_mu.head<3>() - solve_mu.head<3>()).cwiseAbs().sum() / 3.;
        t_error_sq_accum += (model_mu.tail<3>() - solve_mu.tail<3>()).cwiseAbs().sum() / 3.;
      }
    }
    return std::pair<double, double>{std::sqrt(r_error_sq_accum / n), std::sqrt(t_error_sq_accum / n)};
  }

  static void check_maps(const Model &model, const std::vector<MapAndError> &map_and_errors,
                         double tolerance, fvlam::LoggerCout &logger)
  {
    for (auto &map_and_error : map_and_errors) {
      for (auto &m_marker : model.markers_.markers_) {
        auto it = map_and_error.map_->find_marker_const(m_marker.index());
        if (it != nullptr) {
          logger.debug() << "  " << it->to_string();

          REQUIRE(gtsam::assert_equal(fvlam::Transform3::from(m_marker.marker_f_world_).r().rotation_matrix(),
                                      it->t_map_marker().tf().r().rotation_matrix(), tolerance));
          REQUIRE(gtsam::assert_equal(fvlam::Transform3::from(m_marker.marker_f_world_).t().mu(),
                                      it->t_map_marker().tf().t().mu(), tolerance));
        }
      }

      logger.debug() << map_and_error.error_.to_string();
      auto pair = calc_error(model, *map_and_error.map_);
      logger.debug() << "true map error - r:" << pair.first << " t:" << pair.second;
    }
  }

  TEST_CASE("Tmm_test - build_marker_map_tmm from model", "[.][all]")
  {
    TestParams tp;
    fvlam::LoggerCout logger{tp.logger_level};

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

    ModelConfig model_config{create_pose_generator(marker_pose_list_0),
                             create_pose_generator(camera_pose_list_0),
                             camsim::CameraTypes::simulation,
                             0.1775,
                             true};

    Model model{model_config};

    auto solved_maps = run_solvers(model, tp, logger);
    check_maps(model, solved_maps, tp.tolerance, logger);
  }

  TEST_CASE("map_test - Build_marker_map_tmm ring of markers, rotating camera", "[.][all]")
  {
    TestParams tp;
    fvlam::LoggerCout logger{tp.logger_level};

    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{tp.n_markers, 2.},
                             PoseGens::SpinAboutZAtOriginFacingOut{tp.n_cameras},
                             camsim::CameraTypes::simulation,
                             0.1775};

    Model model{model_config};

    auto solved_maps = run_solvers(model, tp, logger);
    check_maps(model, solved_maps, tp.tolerance, logger);
  }

  TEST_CASE("map-test - build_marker_map_tmm circle of markers, camera in circle", "[.][all]")
  {
    TestParams tp;
    fvlam::LoggerCout logger{tp.logger_level};

    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingAlongZ{tp.n_markers, 2., 2., false},
                             PoseGens::CircleInXYPlaneFacingAlongZ{tp.n_cameras, 2., 0., true},
                             camsim::CameraTypes::simulation,
                             0.1775};

    Model model{model_config};

    auto solved_maps = run_solvers(model, tp, logger);
    check_maps(model, solved_maps, tp.tolerance, logger);
  }


// ==============================================================================
// BuildMarkerMapTest class
// ==============================================================================

  class BuildMarkerMapTest
  {
  public:
    struct Config
    {
      int n_markers_;
      int n_cameras_;

      bool average_on_space_not_manifold_;
      bool use_shonan_initial_;
      fvlam::BuildMarkerMapTmmContext::NoiseStrategy noise_strategy_;

      double r_sampler_sigma_;
      double t_sampler_sigma_;
      double u_sampler_sigma_;
      double r_noise_sigma_;
      double t_noise_sigma_;
      double u_noise_sigma_;

      double tolerance_;

      fvlam::Logger::Levels logger_level_;

      explicit Config(int n_markers = 8,
                      int n_cameras = 64,
                      bool average_on_space_not_manifold = false,
                      bool use_shonan_initial = false,
                      fvlam::BuildMarkerMapTmmContext::NoiseStrategy noise_strategy =
                      fvlam::BuildMarkerMapTmmContext::NoiseStrategy::minimum,
                      double r_sampler_sigma = 0.0,
                      double t_sampler_sigma = 0.0,
                      double u_sampler_sigma = 0.0,
                      double r_noise_sigma = 0.1,
                      double t_noise_sigma = 0.3,
                      double u_noise_sigma = 0.5,
                      double tolerance = 4.0e-1,
                      fvlam::Logger::Levels logger_level = fvlam::Logger::Levels::level_info) :
        n_markers_{n_markers},
        n_cameras_{n_cameras},
        average_on_space_not_manifold_{average_on_space_not_manifold},
        use_shonan_initial_{use_shonan_initial},
        noise_strategy_{noise_strategy},
        r_sampler_sigma_{r_sampler_sigma},
        t_sampler_sigma_{t_sampler_sigma},
        u_sampler_sigma_{u_sampler_sigma},
        r_noise_sigma_{r_noise_sigma},
        t_noise_sigma_{t_noise_sigma},
        u_noise_sigma_{u_noise_sigma},
        tolerance_{tolerance},
        logger_level_{logger_level}
      {}
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
    }

    void operator()(std::unique_ptr<fvlam::BuildMarkerMapInterface>)
    {

    }

    auto &marker_observations_list_perturbed() const
    { return marker_observations_list_perturbed_; }
  };


  TEST_CASE("fvlam::Model test", "[all]")
  {
    auto bmm_test_config = BuildMarkerMapTest::Config();

    fvlam::LoggerCout logger{bmm_test_config.logger_level_};

    auto model_maker = [&bmm_test_config]() -> fvlam::MarkerModel
    {
      fvlam::MarkerModel model(fvlam::MapEnvironmentGen::Default(),
                               fvlam::CameraInfoMapGen::DualCamera(),
                               fvlam::CamerasGen::SpinAboutZAtOriginFacingOut(bmm_test_config.n_cameras_),
                               fvlam::MarkersGen::CircleInXYPlaneFacingOrigin(bmm_test_config.n_markers_, 2));
      return model;
    };

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
      map_initial->add_marker(model.targets()[0]);

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

    runner(uut_maker);
  }
}
