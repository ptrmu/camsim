#pragma once

#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"


namespace fvlam
{

  struct MapEnvironmentGen
  {
    static MapEnvironment Default(); //
  };

  struct CameraInfoMapGen
  {
    static CameraInfoMap DualCamera(); //
  };

  struct CamerasGen
  {
    static std::vector<Transform3> SpinAboutZAtOriginFacingOut(int n); //
    static std::vector<Transform3> LookingDownZ(double z); //
  };

  template<class Target>
  struct TargetsGen
  {
    static std::vector<Target> CircleInXYPlaneFacingOrigin(int n, double radius); //
    static std::vector<Target> OriginLookingUp(); //
  };

  using MarkersGen = TargetsGen<Marker>;

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  class MarkerObservations
  {
    uint64_t camera_index_;
    Transform3 t_map_camera_;
    ObservationsSynced observations_synced_;

    static ObservationsSynced gen_observations_synced(const MapEnvironment &map_environment,
                                                      const CameraInfoMap &camera_info_map,
                                                      const Transform3 &t_map_camera,
                                                      const std::vector<Marker> &markers);

  public:
    MarkerObservations(const MapEnvironment &map_environment,
                       const CameraInfoMap &camera_info_map,
                       Transform3 t_map_camera,
                       const std::vector<Marker> &markers,
                       uint64_t camera_index) :
      camera_index_{camera_index},
      t_map_camera_{std::move(t_map_camera)},
      observations_synced_{gen_observations_synced(map_environment, camera_info_map, t_map_camera, markers)}
    {}

    auto &camera_index() const
    { return camera_index_; }

    auto &t_map_camera() const
    { return t_map_camera_; }

    auto &observations_synced() const
    { return observations_synced_; }
  };

// ==============================================================================
// Model class
// ==============================================================================

  template<class Environment, class Target, class TargetObservations>
  class Model
  {
    Environment environment_;
    CameraInfoMap camera_info_map_;
    std::vector<Target> targets_;
    std::vector<TargetObservations> target_observations_list_;

    static std::vector<TargetObservations> gen_target_observations_list(const Environment &environment,
                                                                        const CameraInfoMap &camera_info_map,
                                                                        const std::vector<Transform3> &t_map_cameras,
                                                                        const std::vector<Target> &targets)
    {
      std::vector<TargetObservations> target_observations_list;
      for (size_t i = 0; i < t_map_cameras.size(); i += 1) {
        target_observations_list.template emplace_back(
          TargetObservations(environment, camera_info_map, t_map_cameras[i], targets, i));
      }
      return target_observations_list;
    }

  public:
    Model(Environment environment,
          CameraInfoMap camera_info_map,
          const std::vector<Transform3> &t_map_cameras,
          std::vector<Target> targets) :
      environment_{std::move(environment)},
      camera_info_map_{std::move(camera_info_map)},
      targets_{std::move(targets)},
      target_observations_list_{gen_target_observations_list(environment_, camera_info_map_, t_map_cameras, targets_)}
    {}

    using Maker = std::function<Model<Environment, Target, TargetObservations>(void)>;

    auto &environment() const
    { return environment_; }

    auto &camera_info_map() const
    { return camera_info_map_; }

    auto &targets() const
    { return targets_; }

    auto &target_observations_list() const
    { return target_observations_list_; }
  };

  using MarkerModel = Model<MapEnvironment, Marker, MarkerObservations>;

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
      Test test{test_config_maker_(), model_};
      test(uut);
    }
  };
}