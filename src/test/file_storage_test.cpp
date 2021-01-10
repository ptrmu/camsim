
#include "catch2/catch.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_bundle.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "../../src/build_marker_map_runner.hpp"
#include "../../src/model.hpp"

namespace camsim
{
#if 1
#if 0
  TEST_CASE("file storage test - Load MarkerMap")
  {
    fvlam::LoggerCout logger{fvlam::Logger::level_debug};
    auto map = fvlam::MarkerMap::load("../src/data/three_circles_of_markers_map.yaml", logger);
    logger.debug() << map.to_string();
  }

  TEST_CASE("file storage test - save MarkerMap")
  {
    fvlam::LoggerCout logger{fvlam::Logger::level_debug};
    fvlam::MarkerMap map{0.321};
    map.add_marker(fvlam::Marker{});
    map.add_marker(fvlam::Marker{3, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3), fvlam::Translate3{1, 2, 3}}}, true});

    map.save("marker_map", logger);
    auto loaded_map = fvlam::MarkerMap::load("marker_map", logger);
    logger.debug() << loaded_map.to_string();
  }
#endif

  TEST_CASE("file storage test - save ObservationsBundles")
  {
    fvlam::LoggerCout logger{fvlam::Logger::Levels::level_debug};

    ModelConfig model_config{PoseGens::CircleInXYPlaneFacingOrigin{2, 2.},
                             PoseGens::SpinAboutZAtOriginFacingOut{2},
                             camsim::CameraTypes::simulation,
                             0.1775};
    Model model{model_config};


    auto runner_config = BuildMarkerMapRunnerConfig{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(0.1),
        fvlam::Translate3::MuVector::Constant(0.3)).finished(),
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(0.1),
        fvlam::Translate3::MuVector::Constant(0.3)).finished(),
      fvlam::Translate2::MuVector::Constant(0.2),
      fvlam::Translate2::MuVector::Constant(0.2),
      false
    };
    BuildMarkerMapRunner bmm_runner{model, runner_config};


    auto map = fvlam::MarkerMap{0.321};
    map.add_marker(fvlam::Marker{});
    map.add_marker(fvlam::Marker{2, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1.1, 1.2, 1.3}}},
                                 true});
    map.add_marker(fvlam::Marker{3, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1, 2, 3}}},
                                 false});


    fvlam::ObservationsBundles bundles{map};
    auto camera_info = fvlam::CameraInfo::from(model.cameras_.calibration_);

    for (auto &observations : bmm_runner.observations_perturbed()) {
      fvlam::ObservationsBundle bundle{camera_info, observations};
      bundles.add_bundle(bundle);
    }

    bundles.save("observations_bundles", logger);
    auto loaded_bundles = fvlam::ObservationsBundles::load("observations_bundles", logger);
    REQUIRE(true == bundles.equals(loaded_bundles));
    logger.debug() << loaded_bundles.to_string();
  }

#endif
}
