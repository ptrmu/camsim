#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

namespace fvlam
{

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

  class CameraInfoInterface; //
  class MarkerMap; //
  class MarkerObservations; //

// An interface used to build maps of markers. This is a common interface to
// several modules that use different techniques to build maps.
  class BuildMarkerMapInterface
  {
  public:
    virtual ~BuildMarkerMapInterface() = default;

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    virtual void add_image_observations(std::unique_ptr<const MarkerObservations> observations,
                                        std::unique_ptr<const CameraInfoInterface> camera_info) = 0;

    // Given the observations that have been added so far create and return a marker_map.
    // On entry to this routine the marker_map contains markers that have fixed locations.
    virtual std::string build_marker_map(MarkerMap &map) = 0;

    // Re-initialize the map_builder. (i.e. through out any data accumulated so far)
    virtual std::string reset(std::string &cmd) = 0;
  };
}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
