#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
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
    static CameraInfoMap Simulation(); //
    static CameraInfoMap Dual(); //
    static CameraInfoMap DualWideAngle(); //
  };

  struct CamerasGen
  {
    static std::vector<Transform3> SpinAboutZAtOriginFacingOut(int n); //
    static std::vector<Transform3> LookingDownZ(double z); //
    static std::vector<Transform3> CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                               bool facing_z_plus_not_z_negative); //
  };

  template<class Target>
  struct TargetsGen
  {
    static std::vector<Target> TargetsFromTransform3s(std::vector<Transform3> transform3s); //
    static std::vector<Target> CircleInXYPlaneFacingOrigin(int n, double radius); //
    static std::vector<Target> OriginLookingUp(double x); //
    static std::vector<Target> OriginLookingUpPlusOne(double x0, double x1); //
    static std::vector<Target> CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                           bool facing_z_plus_not_z_negative); //
  };

  using MarkersGen = TargetsGen<Marker>;

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  class MarkerObservations
  {
    std::uint64_t camera_index_;
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
                       std::uint64_t camera_index) :
      camera_index_{camera_index},
      t_map_camera_{std::move(t_map_camera)},
      observations_synced_(gen_observations_synced(map_environment, camera_info_map, t_map_camera, markers))
    {}

    MarkerObservations(std::uint64_t camera_index,
                       Transform3 t_map_camera,
                       ObservationsSynced observations_synced) :
      camera_index_{camera_index},
      t_map_camera_{std::move(t_map_camera)},
      observations_synced_{std::move(observations_synced)}
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

  struct MarkerModelGen
  {
    static MarkerModel::Maker MonoSpinCameraAtOrigin(); //
    static MarkerModel::Maker DualSpinCameraAtOrigin(); //
    static MarkerModel::Maker MonoParallelGrid(); //
    static MarkerModel::Maker DualParallelGrid(); //
    static MarkerModel::Maker MonoParallelCircles(); //
    static MarkerModel::Maker DualParallelCircles(); //
    static MarkerModel::Maker DualWideSingleCamera(); //
    static MarkerModel::Maker DualWideSingleMarker(); //
    static MarkerModel::Maker MonoSingleMarker(); //
    static MarkerModel::Maker DualSingleMarker(); //
    static MarkerModel::Maker MonoDoubleMarker(); //
  };

  struct ModelKey
  {
    const static unsigned char key_char_value = 'a';
    const static unsigned char key_char_camera = 'c';
    const static unsigned char key_char_camera_marker = 'd';
    const static unsigned char key_char_marker_marker = 'e';
    const static unsigned char key_char_corner0 = 'i';
    const static unsigned char key_char_corner1 = 'j';
    const static unsigned char key_char_corner2 = 'k';
    const static unsigned char key_char_corner3 = 'l';
    const static unsigned char key_char_marker = 'm';

    static std::uint64_t value(std::size_t value_idx); // Generic variable
    static std::uint64_t camera(std::size_t camera_idx); //
    static std::uint64_t marker(std::size_t marker_idx); //
    static std::uint64_t camera_marker(std::size_t camera_idx, std::size_t marker_idx); //
    static std::uint64_t marker_marker(std::size_t marker0_idx, std::size_t marker1_idx); //
    static std::uint64_t corner(std::uint64_t marker_key, std::size_t corner_idx); //
    static std::uint64_t marker_from_corner(std::uint64_t corner_key); //
    static std::size_t camera_idx_from_camera_marker(std::uint64_t camera_marker_key); //
    static std::size_t marker_idx_from_camera_marker(std::uint64_t camera_marker_key); //
    static std::size_t id0_from_marker_marker(std::uint64_t marker_marker_key); //
    static std::size_t id1_from_marker_marker(std::uint64_t marker_marker_key); //
  };

// ==============================================================================
// Runner class
// ==============================================================================

#if 0
  class MarkerModelRunnerSimple
  {
  public:
    struct Config
    {
      double equals_tolerance_ = 1.0e-2;
      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

    template<class Uut>
    using UutMaker = std::function<Uut(MarkerModelRunnerSimple &)>;

  private:
    Config cfg_;
    LoggerCout logger_;
    MarkerModel model_;

  public:
    MarkerModelRunnerSimple() = delete; //
    MarkerModelRunnerSimple(const MarkerModelRunnerSimple &) = delete; //
    MarkerModelRunnerSimple(MarkerModelRunnerSimple &&) = delete;

    MarkerModelRunnerSimple(Config cfg,
                            MarkerModel::Maker model_maker);

    template<class TestMaker>
    bool run(TestMaker test_maker)
    {
      auto test = test_maker(*this);
      return test();
    }

    template<class TestMaker, class UutMaker>
    bool run(TestMaker test_maker, UutMaker uut_maker)
    {
      auto test = test_maker(*this);
      auto uut = uut_maker(*this);
      return test(std::move(uut));
    }

    auto &cfg() const
    { return cfg_; }

    Logger &logger()
    { return logger_; }

    auto &model() const
    { return model_; }
  };
#endif

  class MarkerModelRunner
  {
  public:
    struct Config
    {
      double r_sampler_sigma_ = 0.001;
      double t_sampler_sigma_ = 0.001;
      double u_sampler_sigma_ = 0.001;

      double equals_tolerance_ = 1.0e-6;

      fvlam::Logger::Levels logger_level_ = fvlam::Logger::Levels::level_warn;
    };

    template<class Uut>
    using UutMaker = std::function<Uut(MarkerModelRunner &)>;
    using This = MarkerModelRunner;

  private:
    Config cfg_;
    LoggerCout logger_;
    MarkerModel model_;
    fvlam::MarkerMap map_;
    std::vector<Marker> markers_perturbed_;
    std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed_;

    static fvlam::MarkerMap gen_map(
      const fvlam::MarkerModel &model);

    static std::vector<Marker> gen_markers_perturbed(
      const fvlam::MarkerModel &model,
      double r_sampler_sigma,
      double t_sampler_sigma);

    static std::vector<fvlam::MarkerObservations> gen_marker_observations_list_perturbed(
      const fvlam::MarkerModel &model,
      double r_sampler_sigma,
      double t_sampler_sigma,
      double point2_sampler_sigma);

  public:
    MarkerModelRunner() = delete; //
    MarkerModelRunner(const MarkerModelRunner &) = delete; //
    MarkerModelRunner(MarkerModelRunner &&) = delete;

    MarkerModelRunner(Config cfg,
                      MarkerModel::Maker model_maker);

    template<class TestMaker, class UutMaker>
    bool run(TestMaker test_maker, UutMaker uut_maker)
    {
      auto test = test_maker(*this);
      auto uut = uut_maker(*this);
      return test(std::move(uut));
    }

    // For Test class that doesn't have a UnitUnderTest
    template<class TestMaker>
    bool run(TestMaker test_maker)
    {
      auto test = test_maker(*this);
      return test();
    }

    // For Test class that doesn't have a Config or a UnitUnderTest.
    template<class Test>
    static int runner_run(const fvlam::MarkerModelRunner::Config &runner_config,
                          const fvlam::MarkerModel::Maker &model_maker)
    {
      auto marker_runner = This(runner_config, model_maker);
      auto test = Test(marker_runner);
      return test();
    }

    // For Test class that has a Config but not a UnitUnderTest.
    template<class Test>
    static int runner_run(const fvlam::MarkerModelRunner::Config &runner_config,
                          const fvlam::MarkerModel::Maker &model_maker,
                          const typename Test::Config &test_config)
    {
      auto marker_runner = This(runner_config, model_maker);
      auto test = Test(test_config, marker_runner);
      return test();
    }

    auto &cfg() const
    { return cfg_; }

    Logger &logger()
    { return logger_; }

    auto &model() const
    { return model_; }

    auto &map() const
    { return map_; }

    auto &marker_observations_list_perturbed() const
    { return marker_observations_list_perturbed_; }

    int for_each_marker_observations(bool truth_not_perturbed,
                                     std::function<int(const fvlam::MarkerObservations &)>); //
    int for_each_observations(bool truth_not_perturbed,
                              std::function<int(const fvlam::MarkerObservations &,
                                                const fvlam::Observations &,
                                                const fvlam::CameraInfo &)>); //
    int for_each_observation(bool truth_not_perturbed,
                             std::function<int(const fvlam::MarkerObservations &,
                                               const fvlam::Observations &,
                                               const fvlam::CameraInfo &,
                                               const fvlam::Observation &)>); //
    int for_each_corner_f_image(bool truth_not_perturbed,
                                std::function<int(const fvlam::MarkerObservations &,
                                                  const fvlam::Observations &,
                                                  const fvlam::CameraInfo &,
                                                  const fvlam::Observation &,
                                                  std::size_t corner_index,
                                                  const fvlam::Translate2)>); //

    class ForAllMarkerObservations
    {
      std::vector<MarkerObservations>::const_iterator next_;
      std::vector<MarkerObservations>::const_iterator end_;

    public:
      explicit ForAllMarkerObservations(const std::vector<MarkerObservations> &marker_observations_list) :
        next_{marker_observations_list.begin()},
        end_{marker_observations_list.end()}
      {}

      bool test() const
      { return next_ != end_; }

      void next()
      { ++next_; }

      const auto &marker_observations() const
      { return *next_; }
    };

    ForAllMarkerObservations make_for_all_marker_observations(bool truth_not_perturbed)
    {
      return ForAllMarkerObservations{truth_not_perturbed ?
                                      model_.target_observations_list() :
                                      marker_observations_list_perturbed_};
    }

    class ForAllObservations
    {
      const CameraInfoMap &camera_info_map_;
      std::vector<Observations>::const_iterator next_;
      std::vector<Observations>::const_iterator end_;
      const CameraInfo *camera_info_;

      const CameraInfo *find_camera_info()
      {
        for (; test(); ++next_) {
          // Find the camera_info
          const auto &kp = camera_info_map_.m().find(next_->imager_frame_id());
          if (kp != camera_info_map_.m().end()) {
            return &kp->second;
          }
        }
        return nullptr;
      }

    public:
      ForAllObservations(const MarkerObservations &marker_observations,
                         const CameraInfoMap &camera_info_map) :
        camera_info_map_{camera_info_map},
        next_{marker_observations.observations_synced().v().begin()},
        end_{marker_observations.observations_synced().v().end()},
        camera_info_{find_camera_info()}
      {}

      bool test() const
      { return next_ != end_; }

      void next()
      {
        ++next_;
        camera_info_ = find_camera_info();
      }

      const auto &observations() const
      { return *next_; }

      const auto &camera_info() const
      { return *camera_info_; }
    };

    class ForAllObservation
    {
      std::vector<Observation>::const_iterator next_;
      std::vector<Observation>::const_iterator end_;

    public:
      explicit ForAllObservation(const Observations &observations) :
        next_{observations.v().begin()},
        end_{observations.v().end()}
      {}

      bool test() const
      { return next_ != end_; }

      void next()
      { ++next_; }

      const auto &observation() const
      { return *next_; }
    };

    class ForAllCornerFImage
    {
      std::size_t corner_index_;
      Observation::Array corners_f_image_;

    public:
      explicit ForAllCornerFImage(Observation::Array corners_f_image) :
        corner_index_{0},
        corners_f_image_{std::move(corners_f_image)}
      {}

      bool test() const
      { return corner_index_ < Observation::ArraySize; }

      void next()
      { ++corner_index_; }

      const auto &corner_index() const
      { return corner_index_; }

      const auto &corner_f_image() const
      { return corners_f_image_[corner_index_]; }
    };

    class ForAllObservationPair
    {
      const std::vector<Observation> &v_;
      std::size_t i0_;
      std::size_t i1_;

    public:
      explicit ForAllObservationPair(const Observations &observations) :
        v_{observations.v()}, i0_{0}, i1_{1}
      {}

      bool test() const
      { return i0_ < v_.size() && i1_ < v_.size(); }

      void next()
      {
        i1_ += 1;
        if (i1_ >= v_.size()) {
          i0_ += 1;
          i1_ = i0_ + 1;
        }
      }

      const auto i0() const
      { return i0_; }

      const auto i1() const
      { return i1_; }

      const auto &observation0() const
      { return v_[i0_]; }

      const auto &observation1() const
      { return v_[i1_]; }
    };


    int for_all_marker_observations(bool truth_not_perturbed,
                                    const std::function<int(const MarkerObservations &)> &); //
    int for_all_observations(bool truth_not_perturbed,
                             const std::function<int(const MarkerObservations &,
                                                     const Observations &,
                                                     const CameraInfo &)> &); //
    int for_all_observation(bool truth_not_perturbed,
                            const std::function<int(const MarkerObservations &,
                                                    const Observations &,
                                                    const CameraInfo &,
                                                    const Observation &)> &); //
    int for_all_corner_f_image(bool truth_not_perturbed,
                               const std::function<int(const MarkerObservations &,
                                                       const Observations &,
                                                       const CameraInfo &,
                                                       const Observation &,
                                                       std::size_t corner_index,
                                                       const Translate2)> &); //
  };
}
