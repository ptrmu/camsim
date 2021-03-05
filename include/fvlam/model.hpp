#pragma once

#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"

namespace fvlam
{

// ==============================================================================
// Model class
// ==============================================================================

  template<class Target>
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

  using MarkerModel = Model<Marker>;

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