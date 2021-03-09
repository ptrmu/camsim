
#include "catch2/catch.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_series.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "../../src/build_marker_map_runner.hpp"
#include "../../src/model.hpp"

namespace fvlam
{
  template<>
  BuildMarkerMapRecorderContext BuildMarkerMapRecorderContext::from<const std::string>(const std::string &other)
  {
    return BuildMarkerMapRecorderContext(other);
  }
}

namespace camsim
{
  TEST_CASE("file storage test - Load MarkerMap", "[.][all]")
  {
    fvlam::LoggerCout logger{fvlam::Logger::level_info};
    auto map = fvlam::MarkerMap::load("../src/data/three_circles_of_markers_map.yaml", logger);
    logger.debug() << map.to_string();

    auto tf = fvlam::Transform3{fvlam::Rotate3::RzRyRx(M_PI / 2., 0, M_PI / 4.),
                                fvlam::Translate3{-2 * cos(45 * M_PI / 180), 2 * cos(45 * M_PI / 180), -1}};
    fvlam::Marker expected{9, fvlam::Transform3WithCovariance{tf}, true};
    auto actual_it = map.find_marker(expected.id());
    REQUIRE(actual_it != nullptr);
    REQUIRE(expected.equals(*actual_it));
  }

  TEST_CASE("file storage test - save MarkerMap", "[.][all]")
  {
    fvlam::LoggerCout logger{fvlam::Logger::level_info};
    fvlam::MarkerMap map{fvlam::MapEnvironment{"", 0, 0.321}};
    map.add_marker(fvlam::Marker{});
    map.add_marker(fvlam::Marker{3, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3), fvlam::Translate3{1, 2, 3}}}, true});

    map.save("marker_map", logger);
    auto loaded_map = fvlam::MarkerMap::load("marker_map", logger);
    logger.debug() << loaded_map.to_string();
    REQUIRE(map.equals(loaded_map));
  }

  TEST_CASE("file storage test - save ObservationsSeries", "[.][all]")
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


    auto map = fvlam::MarkerMap{fvlam::MapEnvironment{"", 0, 0.321}};
    map.add_marker(fvlam::Marker{});
    map.add_marker(fvlam::Marker{2, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1.1, 1.2, 1.3}}},
                                 true});
    map.add_marker(fvlam::Marker{3, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1, 2, 3}}},
                                 false});


    auto camera_info = fvlam::CameraInfo::from(model.cameras_.calibration_);
    auto camera_info_map = fvlam::CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info.imager_frame_id(), camera_info);

    auto observations_series = fvlam::ObservationsSeries{map, camera_info_map};

    for (auto &observations : bmm_runner.observations_perturbed()) {
      auto observations_synched = fvlam::ObservationsSynced{fvlam::Stamp{}, "camera"};
      observations_synched.v_mutable().emplace_back(observations);
      observations_series.v_mutable().emplace_back(observations_synched);
    }

    auto filename = std::string("observations_series");
    observations_series.save(filename, logger);
    auto loaded_series = fvlam::ObservationsSeries::load(filename, logger);
    REQUIRE(true == observations_series.equals(loaded_series));
    logger.debug() << loaded_series.to_string();
  }

  TEST_CASE("file storage test - build_marker_map_recorder", "[.][all]")
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


    auto map = fvlam::MarkerMap{fvlam::MapEnvironment{"", 0, 0.321}};
    map.add_marker(fvlam::Marker{});
    map.add_marker(fvlam::Marker{2, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1.1, 1.2, 1.3}}},
                                 true});
    map.add_marker(fvlam::Marker{3, fvlam::Transform3WithCovariance{
      fvlam::Transform3{fvlam::Rotate3::RzRyRx(0.1, 0.2, 0.3),
                        fvlam::Translate3{1, 2, 3}}},
                                 false});

    // Use the recorder to record observations series
    auto bmm_recorder_context = fvlam::BuildMarkerMapRecorderContext::from<const std::string>(
      std::string{"bmm_recorder"});
    auto bmm_recorder = make_build_marker_map(bmm_recorder_context, logger, map);
    auto built_map = bmm_runner(*bmm_recorder);
    bmm_recorder.reset(nullptr); // releasing the recorder closes the file

    auto camera_info = fvlam::CameraInfo::from(model.cameras_.calibration_);
    auto camera_info_map = fvlam::CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info.imager_frame_id(), camera_info);

    auto observations_series = fvlam::ObservationsSeries{map, camera_info_map};

    for (auto &observations : bmm_runner.observations_perturbed()) {
      auto observations_synched = fvlam::ObservationsSynced{fvlam::Stamp{}, "camera"};
      observations_synched.v_mutable().emplace_back(observations);
      observations_series.v_mutable().emplace_back(observations_synched);
    }

    // Compare the series recorded by the recorder with the series created directly from the model.
    auto loaded_series = fvlam::ObservationsSeries::load("bmm_recorder", logger);
    REQUIRE(true == observations_series.equals(loaded_series));
    logger.debug() << loaded_series.to_string();
  }
}
