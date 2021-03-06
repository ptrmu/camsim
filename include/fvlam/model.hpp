#pragma once

#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "opencv2/core.hpp"


namespace fvlam
{

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  class MarkerObservations
  {
    uint64_t camera_index_;
    ObservationsSynced observations_synced_;

    static ObservationsSynced gen_observations_synced(Transform3 t_map_camera,
                                                      const CameraInfoMap &camera_info_map,
                                                      const std::vector<Marker> markers,
                                                      double marker_length)
    {
      auto observations_synced = ObservationsSynced{Stamp{}, "camera_frame"};

      for (auto &camera_info_pair : camera_info_map) {
        const CameraInfo &camera_info = camera_info_pair.second;
        auto cv_camera_calibration = camera_info.to<fvlam::CvCameraCalibration>();
        auto cv_project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
          cv_camera_calibration,
          fvlam::Transform3::from(t_map_camera),
          marker_length);

        auto observations = Observations{camera_info.imager_frame_id()};
        for (auto &marker : markers) {

        }
      }
    }

  public:
    MarkerObservations(uint64_t camera_index,
                       Transform3 t_map_camera,
                       const CameraInfoMap &camera_info_map,
                       const std::vector<Marker> markers,
                       double marker_length) :
      camera_index_{camera_index},
      observations_synced_{gen_observations_synced(t_map_camera, camera_info_map, markers, marker_length)}
    {}

    auto &camera_index() const
    { return camera_index_; }

    auto &observations_synced() const
    { return observations_synced_; }
  };

// ==============================================================================
// Model class
// ==============================================================================

  template<class Target, class TargetObservations>
  class Model
  {
    MapEnvironment map_environment_;
    CameraInfoMap camera_info_map_;
    std::vector<Camera> cameras_;
    std::vector<Target> targets_;

  public:
    Model(MapEnvironment map_environment,
          CameraInfoMap camera_info_map,
          std::vector<Camera> cameras,
          std::vector<Target> targets) :
      map_environment_{map_environment},
      camera_info_map_{camera_info_map},
      cameras_{cameras},
      targets_{targets}
    {}

    using Maker = std::function<Model(void)>;

    auto &map_environment() const
    { return map_environment_; }

    auto &camera_info_map() const
    { return camera_info_map_; }

    auto &cameras() const
    { return cameras_; }

    auto &targets() const
    { return targets_; }
  };

  using MarkerModel = Model<Marker, MarkerObservations>;

// ==============================================================================
// Runner class
// ==============================================================================

  template<class Model, class Test, class UUT>
  class Runner
  {
    Model model_;
    typename Test::ConfigMaker test_config_maker_;

  public:
    Runner(typename Model::Maker model_maker,
           typename Test::ConfigMaker test_config_maker) :
      model_{model_maker()}, test_config_maker_{test_config_maker}
    {}

    void operator()(UUT &uut)
    {
      Test test{test_config_maker_(), model_, uut};
      test();
    }
  };
}