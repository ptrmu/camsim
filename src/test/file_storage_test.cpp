
#include "catch2/catch.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_bundle.hpp"
#include "fvlam/transform3_with_covariance.hpp"

namespace camsim
{
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
}
