
#include "build_marker_map_runner.hpp"

namespace fvlam
{
  template<>
  Transform3 Transform3::from<const camsim::CameraModel>(const camsim::CameraModel &other)
  {
    return Transform3::from(other.camera_f_world_);
  }

  template<>
  Transform3 Transform3::from<const camsim::MarkerModel>(const camsim::MarkerModel &other)
  {
    return Transform3::from(other.marker_f_world_);
  }

  template<>
  Marker Marker::from<const camsim::MarkerModel>(const camsim::MarkerModel &other)
  {
    return Marker{other.index(), fvlam::Transform3WithCovariance{Transform3::from(other)}};
  }
}

namespace camsim
{
  BuildMarkerMapRunner::BuildMarkerMapRunner(const Model &model,
                                             const BuildMarkerMapRunnerConfig &cfg) :
    model_{model},
    cfg_{std::move(cfg)},
    camera_info_{fvlam::CameraInfo::from(model_.cameras_.calibration_)}
  {
    // Prepare
    auto pose3_sampler = gtsam::Sampler{cfg_.pose3_sampler_sigmas_};
    auto point2_sampler = gtsam::Sampler{cfg_.point2_sampler_sigmas_};

    auto gtsam_camera_info = camera_info_.to<gtsam::Cal3DS2>();

    // build up collections of perturbed measurements
    for (auto &m_camera : model_.cameras_.cameras_) {
      // Create the perturbed camera
      auto t_world_camera_perturbed = fvlam::Transform3WithCovariance{
        fvlam::Transform3{
          fvlam::Transform3::from<const gtsam::Pose3>(m_camera.camera_f_world_.retract(pose3_sampler.sample()))},
        cfg.pose3_noise_sigmas_.asDiagonal()};
      t_world_cameras_perturbed_.emplace_back(t_world_camera_perturbed);

      // Set up for projecting to get observations
      auto project_function = fvlam::Marker::project_t_world_marker(gtsam_camera_info,
                                                                    fvlam::Transform3::from(m_camera),
                                                                    model_.cfg_.marker_length_);

      fvlam::Observations observations{fvlam::Stamp{}, ""};
      for (auto &m_marker : model_.markers_.markers_) {
        // Create the perturbed marker
        auto t_world_marker_perturbed = fvlam::Marker{
          m_marker.index(),
          fvlam::Transform3WithCovariance{
            fvlam::Transform3{
              fvlam::Transform3::from<const gtsam::Pose3>(m_marker.marker_f_world_.retract(pose3_sampler.sample()))},
            cfg.pose3_noise_sigmas_.asDiagonal()}};
        markers_perturbed_.emplace_back(t_world_marker_perturbed);

        // Create the perturbed Observation
        auto observation = project_function(fvlam::Marker::from(m_marker));
        if (observation.is_valid()) {
          auto cfi = observation.corners_f_image();
          auto corners_f_image_perturbed = fvlam::Observation::Array{
            fvlam::Translate2{cfi[0].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[1].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[2].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[3].t() + point2_sampler.sample()},
          };
          auto observation_perturbed = fvlam::Observation{observation.id(),
                                                          corners_f_image_perturbed,
                                                          cfg.point2_noise_sigmas_.asDiagonal()};

          observations.emplace_back(observation_perturbed);
        }
      }
      observations_perturbed_.emplace_back(observations);
    }
  }

  std::unique_ptr<fvlam::MarkerMap> BuildMarkerMapRunner::operator()(fvlam::BuildMarkerMapInterface &build_map)
  {
    frames_processed_ = 0;

    gttic(solver);

    // Loop over all the cameras
    for (auto &camera : model_.cameras_.cameras_) {

      // Pass the perturbed observations to the builder
      build_map.process(observations_perturbed_[camera.index()], camera_info_);

      this->frames_processed_ += 1;
    }

    // Build the map.
    auto result = build_map.build();

    gttoc(solver);
#ifdef ENABLE_TIMING
    gtsam::tictoc_print();
#endif
    gtsam::tictoc_reset_();

    return result;
  }

}
