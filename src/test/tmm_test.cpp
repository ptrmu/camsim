
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
      const fvlam::Transform3::MuVector pose3_sampler_sigmas_;
      const fvlam::Transform3::MuVector pose3_noise_sigmas_;
      const fvlam::Translate2::MuVector point2_sampler_sigmas_;
      const fvlam::Translate2::MuVector point2_noise_sigmas_;
      const bool print_covariance_{false};

      Config() = default; // Todo remove this

      Config(const fvlam::Transform3::MuVector &pose3_sampler_sigmas,
             const fvlam::Transform3::MuVector &pose3_noise_sigmas,
             const fvlam::Translate2::MuVector &point2_sampler_sigmas,
             const fvlam::Translate2::MuVector &point2_noise_sigmas,
             bool print_covariance) :
        pose3_sampler_sigmas_{pose3_sampler_sigmas},
        pose3_noise_sigmas_{pose3_noise_sigmas},
        point2_sampler_sigmas_{point2_sampler_sigmas},
        point2_noise_sigmas_{point2_noise_sigmas},
        print_covariance_{print_covariance}
      {}
    };

  private:
    const Model &model_;
    const BuildMarkerMapRunnerConfig cfg_;
    fvlam::CameraInfo camera_info_;

    std::vector<fvlam::Transform3WithCovariance> t_world_cameras_perturbed_{};
    std::vector<fvlam::Marker> markers_perturbed_{};
    std::vector<fvlam::Observations> observations_perturbed_{};
    int frames_processed_{0};

  public:
    using ConfigMaker = std::function<Config(void)>;

    BuildMarkerMapTest(const MarkerModel &model,
                       const Config &cfg);

    std::unique_ptr<fvlam::MarkerMap> operator()(fvlam::BuildMarkerMapInterface &build_map);

    auto &observations_perturbed() const
    { return observations_perturbed_; }
  };


  TEST_CASE("fvlam::Model test", "[all]")
  {
    auto model_maker = []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironment{},
                                fvlam::CameraInfoMap{},
                                std::vector<fvlam::Transform3>{},
                                std::vector<fvlam::Marker>{});
    };

    auto test_config_maker = []() -> BuildMarkerMapTest::Config
    {
      return BuildMarkerMapTest::Config();
    };

    auto runner = fvlam::Runner<fvlam::MarkerModel, BuildMarkerMapTest, fvlam::BuildMarkerMapInterface>{
      model_maker, test_config_maker};

//    auto build_marker_map_interface =
//    runner(build_marker_map_interface);
  }
}
