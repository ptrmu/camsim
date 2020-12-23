#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  class CameraInfo; //
  class MarkerMap; //
  class MarkerObservations; //

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

// An interface used to build maps of markers. This is a common interface to
// several modules that use different techniques to build maps.
  class BuildMarkerMapInterface
  {
  public:
    virtual ~BuildMarkerMapInterface() = default;

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    virtual void process_image_observations(const MarkerObservations &marker_observations,
                                            const CameraInfo &camera_info) = 0;

    // Given the observations that have been added so far create and return a marker_map.
    // On entry to this routine the marker_map contains markers that have fixed locations.
    virtual std::unique_ptr<MarkerMap> build_marker_map() = 0;

    // Re-initialize the map_builder. (i.e. through out any data accumulated so far)
    virtual std::string reset(std::string &cmd) = 0;
  };

  template<class TContext>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map(const TContext &context,
                                                                 std::unique_ptr<MarkerMap> map_initial);

// ==============================================================================
// BuildMarkerMapShonanContext class
// ==============================================================================

  class BuildMarkerMapShonanContext
  {
  public:
    const int flags_;

    explicit BuildMarkerMapShonanContext(int flags) :
      flags_{flags}
    {}

    template<class T>
    static Transform3 from(const T &other);
  };

}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
