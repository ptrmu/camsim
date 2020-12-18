#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  class CameraInfo; //
  class MarkerMap; //
  class MarkerObservation; //
  class MarkerObservations; //

// ==============================================================================
// TransformFromObservationInterface class
// ==============================================================================

// An interface used to convert observations of marker corners into transforms.
  class TransformFromObservationInterface
  {
  public:
    virtual ~TransformFromObservationInterface() = default;

    // calculate the pose of a marker in the camera frame.
    virtual Transform3WithCovariance calc_t_camera_marker(const MarkerObservation &marker_observation,
                                                          const CameraInfo &camera_info) = 0;

    // Calculate the pose of marker1 in the frame of marker0.
    virtual Transform3WithCovariance calc_t_marker0_marker1(const MarkerObservation &marker_observation,
                                                            const CameraInfo &camera_info) = 0;
  };

  std::unique_ptr<TransformFromObservationInterface>
  make_transform_from_observation_solve_pnp(double marker_length);

  std::unique_ptr<TransformFromObservationInterface>
  make_transform_from_observation_resection(double marker_length);

  std::unique_ptr<TransformFromObservationInterface>
  ake_transform_from_observation_custom_factor(double marker_length);

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
    virtual std::string build_marker_map(MarkerMap &map) = 0;

    // Re-initialize the map_builder. (i.e. through out any data accumulated so far)
    virtual std::string reset(std::string &cmd) = 0;
  };

  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map_shonan();
}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
