#ifndef BUILD_MARKER_MAP_RUNNER_HPP
#define BUILD_MARKER_MAP_RUNNER_HPP


#define ENABLE_TIMING

#include "model.hpp"
#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
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

    BuildMarkerMapRunnerConfig(fvlam::Transform3::MuVector pose3_sampler_sigmas,
                               fvlam::Transform3::MuVector pose3_noise_sigmas,
                               fvlam::Translate2::MuVector point2_sampler_sigmas,
                               fvlam::Translate2::MuVector point2_noise_sigmas,
                               bool print_covariance) :
      pose3_sampler_sigmas_{std::move(pose3_sampler_sigmas)},
      pose3_noise_sigmas_{std::move(pose3_noise_sigmas)},
      point2_sampler_sigmas_{std::move(point2_sampler_sigmas)},
      point2_noise_sigmas_{std::move(point2_noise_sigmas_)},
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

    gtsam::Sampler pose3_sampler_{fvlam::Transform3::MuVector::Zero()};
    gtsam::Sampler point2_sampler_{fvlam::Translate2::MuVector::Zero()};
    int frames_processed_{0};

  public:
    BuildMarkerMapRunner(const Model &model,
                         const BuildMarkerMapRunnerConfig &cfg) :
      model_{model},
      cfg_{std::move(cfg)}
    {}

    std::unique_ptr<fvlam::MarkerMap> operator()(fvlam::BuildMarkerMapInterface &build_map);
  };
}
#endif //BUILD_MARKER_MAP_RUNNER_HPP
