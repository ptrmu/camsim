
#include "build_marker_map_runner.hpp"

namespace camsim
{
  std::unique_ptr<fvlam::MarkerMap> BuildMarkerMapRunner::operator()(fvlam::BuildMarkerMapInterface &build_map)
  {
    // Prepare
    pose3_sampler_ = gtsam::Sampler{cfg_.pose3_sampler_sigmas_};
    point2_sampler_ = gtsam::Sampler{cfg_.point2_sampler_sigmas_};
    frames_processed_ = 0;

    auto camera_info = fvlam::CameraInfo::from(model_.cameras_.calibration_);
    auto gtsam_camera_info = camera_info.to<gtsam::Cal3DS2>();

    gttic(solver);

    // Loop over all the cameras
    for (auto &camera : model_.cameras_.cameras_) {

      // Create a set of observations
      auto project_function = fvlam::Marker::project_t_world_marker(gtsam_camera_info,
                                                                    fvlam::Transform3::from(camera),
                                                                    model_.cfg_.marker_size_);
      fvlam::MarkerObservations observations{};
      for (auto &marker : model_.markers_.markers_) {
        auto observation = project_function(fvlam::Marker::from(marker));
        observations.add(observation);
      }

      // Pass these observations to the builder
      build_map.process_image_observations(observations,
                                           camera_info);

      this->frames_processed_ += 1;
    }

    // Build the map.
    auto result = build_map.build_marker_map();

    gttoc(solver);
    gtsam::tictoc_print();
    gtsam::tictoc_reset_();

    return result;
  }

}
