#ifndef BUILD_MARKER_MAP_RUNNER_HPP
#define BUILD_MARKER_MAP_RUNNER_HPP


//#define ENABLE_TIMING

#include "model.hpp"
#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include <gtsam/base/timing.h>
#include <gtsam/linear/NoiseModel.h>
#include "gtsam/linear/Sampler.h"

namespace camsim
{
// ==============================================================================
// BuildMarkerMapRunnerConfig class
// ==============================================================================

  struct BuildMarkerMapRunnerConfig
  {
    const fvlam::Transform3::MuVector pose3_sampler_sigmas_;
    const fvlam::Transform3::MuVector pose3_noise_sigmas_;
    const fvlam::Translate2::MuVector point2_sampler_sigmas_;
    const fvlam::Translate2::MuVector point2_noise_sigmas_;
    const bool print_covariance_;

    BuildMarkerMapRunnerConfig(const fvlam::Transform3::MuVector &pose3_sampler_sigmas,
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

// ==============================================================================
// BuildMarkerMapRunner class
// ==============================================================================

  class BuildMarkerMapRunner
  {
    const Model &model_;
    const BuildMarkerMapRunnerConfig cfg_;
    fvlam::CameraInfo camera_info_;

    std::vector<fvlam::Transform3WithCovariance> t_world_cameras_perturbed_{};
    std::vector<fvlam::Marker> markers_perturbed_{};
    std::vector<fvlam::Observations> observations_perturbed_{};
    int frames_processed_{0};

  public:
    BuildMarkerMapRunner(const Model &model,
                         const BuildMarkerMapRunnerConfig &cfg);

    std::unique_ptr<fvlam::MarkerMap> operator()(fvlam::BuildMarkerMapInterface &build_map);
  };
}
#endif //BUILD_MARKER_MAP_RUNNER_HPP
