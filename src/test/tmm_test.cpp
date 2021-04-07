
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

      double r_noise_sigma_ = 0.1;
      double t_noise_sigma_ = 0.3;
      double u_noise_sigma_ = 0.5;
    };

  private:
    const Config cfg_;
    fvlam::MarkerModelRunner &runner_;

  public:
    using Maker = std::function<BuildMarkerMapTest(fvlam::MarkerModelRunner &)>;
    using UutMaker = std::function<std::unique_ptr<fvlam::BuildMarkerMapInterface>(fvlam::MarkerModelRunner &)>;

    BuildMarkerMapTest(const Config &cfg,
                       fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}
    {}

    bool operator()(std::unique_ptr<fvlam::BuildMarkerMapInterface> build_marker_map)
    {
      // Loop over the list of observations
      for (auto &marker_observation : runner_.marker_observations_list_perturbed()) {

        // Pass the perturbed observations to the builder
        build_marker_map->process(marker_observation.observations_synced(), runner_.model().camera_info_map());
      }

      // Build the map.
      auto built_map = build_marker_map->build();
      auto error = fvlam::BuildMarkerMapTmmContext::BuildError::from(*build_marker_map, *built_map);

      runner_.logger().info() << "Resulting Markers:\n"
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

      for (auto &model_marker : runner_.model().targets()) {
        auto it = built_map.find_marker_const(model_marker.id());
        if (it == nullptr) {
          return false;
        }

        runner_.logger().debug() << "  " << it->to_string();
        if (!model_marker.t_map_marker().tf().equals(it->t_map_marker().tf(), runner_.cfg().equals_tolerance_, true)) {
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
      runner_.logger().info() << "True map error - r:" << r_error << " t:" << t_error;
      return true;
    }
  };

  static bool test_build_marker_map_tmm(
    fvlam::MarkerModelRunner::Config runner_config,
    BuildMarkerMapTest::Config bmm_test_config,
    fvlam::MarkerModel::Maker model_maker)
  {
    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto marker_runner = fvlam::MarkerModelRunner(runner_config, model_maker);

    auto test_maker = [&bmm_test_config](fvlam::MarkerModelRunner &runner) -> BuildMarkerMapTest
    {
      return BuildMarkerMapTest(bmm_test_config, runner);
    };

    auto uut_maker = [&bmm_test_config](
      fvlam::MarkerModelRunner &runner) -> std::unique_ptr<fvlam::BuildMarkerMapInterface>
    {
      auto map_initial = std::make_unique<fvlam::MarkerMap>(runner.model().environment());
      map_initial->add_marker(fvlam::Marker(runner.model().targets()[0].id(),
                                            runner.model().targets()[0].t_map_marker(),
                                            true));

      auto solve_tmm_context = fvlam::SolveTmmContextCvSolvePnp{bmm_test_config.average_on_space_not_manifold_};
      auto solve_tmm_factory = fvlam::make_solve_tmm_factory(solve_tmm_context,
                                                             runner.model().environment().marker_length());

      auto tmm_context = fvlam::BuildMarkerMapTmmContext(solve_tmm_factory,
                                                         bmm_test_config.use_shonan_initial_,
                                                         bmm_test_config.noise_strategy_,
                                                         bmm_test_config.r_noise_sigma_,
                                                         bmm_test_config.t_noise_sigma_);
      return make_build_marker_map(tmm_context, runner.logger(), *map_initial);
    };

    return marker_runner.run<BuildMarkerMapTest::Maker, BuildMarkerMapTest::UutMaker>(test_maker, uut_maker);
  }

  static void test_four_build_marker_map_tmm(
    fvlam::MarkerModelRunner::Config runner_config,
    BuildMarkerMapTest::Config bmm_test_config,
    fvlam::MarkerModel::Maker model_maker)
  {
    bmm_test_config.average_on_space_not_manifold_ = false;
    bmm_test_config.use_shonan_initial_ = false;
    REQUIRE(test_build_marker_map_tmm(runner_config, bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = true;
    bmm_test_config.use_shonan_initial_ = false;
    REQUIRE(test_build_marker_map_tmm(runner_config, bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = false;
    bmm_test_config.use_shonan_initial_ = true;
    REQUIRE(test_build_marker_map_tmm(runner_config, bmm_test_config, model_maker));
    bmm_test_config.average_on_space_not_manifold_ = true;
    bmm_test_config.use_shonan_initial_ = true;
    REQUIRE(test_build_marker_map_tmm(runner_config, bmm_test_config, model_maker));
  }

  TEST_CASE("build_marker_map_tmm - rotating camera - match markers", "[all]")
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto bmm_test_config = BuildMarkerMapTest::Config();

    test_four_build_marker_map_tmm(runner_config, bmm_test_config, fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
  }

  TEST_CASE("build_marker_map_tmm - space accumulate - match markers", "[all]")
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto bmm_test_config = BuildMarkerMapTest::Config();

    test_four_build_marker_map_tmm(runner_config, bmm_test_config, fvlam::MarkerModelGen::DualWideSingleCamera());
  }

  TEST_CASE("build_marker_map_tmm - build_marker_map_tmm circle of markers, camera in circle", "[all]")
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto bmm_test_config = BuildMarkerMapTest::Config();

    test_four_build_marker_map_tmm(runner_config, bmm_test_config, fvlam::MarkerModelGen::MonoParallelCircles());
  }

  TEST_CASE("build_marker_map_tmm - build_marker_map_tmm from model", "[all]")
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto bmm_test_config = BuildMarkerMapTest::Config();

    test_four_build_marker_map_tmm(runner_config, bmm_test_config, fvlam::MarkerModelGen::MonoParallelGrid());
  }
}
