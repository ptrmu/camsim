#pragma ide diagnostic ignored "modernize-use-nodiscard"

//#define ENABLE_TIMING

#include "smf_run.hpp"
#include "cal_info.hpp"
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/model.hpp"
#include "gtsam/base/debug.h"
#include <gtsam/base/timing.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

namespace camsim
{

  struct GraphData
  {
    gtsam::Key t_m0_m1_key_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;

    explicit GraphData(gtsam::Key t_m0_m1_key) :
      t_m0_m1_key_{t_m0_m1_key},
      graph_{}, initial_{}
    {}
  };

  struct FixedLagData
  {
    gtsam::Key t_m0_m1_key_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values initial_;
    gtsam::FixedLagSmoother::KeyTimestampMap timestamps_;
    gtsam::BatchFixedLagSmoother batch_smoother_;
    std::uint64_t last_camera_index_{std::numeric_limits<std::uint64_t>::max()};
    double current_timestamp_{0.};

    FixedLagData(gtsam::Key t_m0_m1_key, double lag,
                 const gtsam::LevenbergMarquardtParams &params) :
      t_m0_m1_key_{t_m0_m1_key},
      graph_{}, initial_{},
      timestamps_{},
      batch_smoother_{lag, params}
    {}
  };

// ==============================================================================
// InterMarkerPoseTest class
// ==============================================================================

  class InterMarkerPoseTest
  {
  public:
    using This = InterMarkerPoseTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

    struct Config
    {
      int algorithm_ = 1; // 0 - single marker, 1 - multiple markers
    };

  private:
    Config cfg_;
    fvlam::MarkerModelRunner &runner_;

    const gtsam::SharedNoiseModel measurement_noise_;
    std::array<gtsam::Point3, 4> corners_f_marker_;

    template<class TData>
    TData &add_or_lookup(std::uint64_t key,
                         std::map<std::uint64_t, TData> &map,
                         const std::function<TData(std::uint64_t)> &make_data)
    {
      auto pair = map.find(key);
      if (pair != map.end()) {
        return pair->second;
      }

      map.emplace(key, make_data(key));
      return map.find(key)->second;
    }

    template<class TData>
    TData &add_marker_pair_to_graph(
      const fvlam::MarkerObservations &marker_observations,
      const fvlam::CameraInfo &camera_info,
      const fvlam::Observation &observation0,
      const fvlam::Observation &observation1,
      std::map<std::uint64_t, TData> &map,
      const std::function<TData(std::uint64_t)> &make_data)
    {
      auto t_m0_c_key = fvlam::ModelKey::camera(marker_observations.camera_index());
      auto t_m0_m1_key = fvlam::ModelKey::marker_marker(observation0.id(), observation1.id());

      auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());
      auto t_camera_imager = camera_info.t_camera_imager().is_valid() ?
                             std::optional<gtsam::Pose3>(camera_info.t_camera_imager().to<gtsam::Pose3>()) :
                             std::nullopt;

      auto m0_corners_f_image = observation0.to<std::array<gtsam::Point2, 4>>();
      auto m1_corners_f_image = observation1.to<std::array<gtsam::Point2, 4>>();

      auto &data = add_or_lookup(t_m0_m1_key, map, make_data);

      // Add the factor to the graph
      data.graph_.template emplace_shared<QuadMarker0Marker1Factor>(
        t_m0_c_key, t_m0_m1_key,
        m0_corners_f_image,
        m1_corners_f_image,
        measurement_noise_,
        corners_f_marker_,
        t_camera_imager,
        cal3ds2, runner_.logger(), "Single Pair Test");

      // Deciding to add the initial value depends on if this is fixed lag or full smoothing.
      // Full Smoothing:
      //  Add each initial value if it doesn't exist.
      // Fixed Lag Smoothing:
      //  The graph and initial structures contain new factors that are will be added to the
      //  optimization system. The initial values should only be added here if they do not
      //  already exist in the optimization system.
      //    t_m0_m1 - This should be added when the optimization system is first created
      //    t_m0_c - Since there could be multiple imagers, multiple measurements could be
      //      made with the same camera. This initial value should only be added for the
      //      first measurement for this camera. Determining this is more complicated.
      //      Presumably all imagers are processed together so keep a record of the last
      //      camera_index. When it changes, a t_m0_c initial variable has to be specified.

      bool add_t_m0_c{false};
      bool add_t_m0_m1{false};

      if constexpr (!std::is_same_v<TData, FixedLagData>) {
        add_t_m0_c = !data.initial_.exists(t_m0_c_key);
        add_t_m0_m1 = !data.initial_.exists(t_m0_m1_key);

      } else {
        if (data.last_camera_index_ == std::numeric_limits<std::uint64_t>::max()) {
          add_t_m0_m1 = true;
        }
        if (data.last_camera_index_ != marker_observations.camera_index()) {
          add_t_m0_c = true;
          data.last_camera_index_ = marker_observations.camera_index();

          // The current_timestamp is updated everytime there is a an update
          // done on this data system. In this way we smooth over the last
          // "lag" measurements for this particular marker pair.
          data.timestamps_[t_m0_c_key] = data.current_timestamp_;
        }
      }

      if (add_t_m0_c || add_t_m0_m1) {
        gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.35, 0.2, 0.25), gtsam::Point3(0.5, -0.10, 0.20));
//        gtsam::Pose3 delta{};

        auto t_w_m0 = runner_.model().targets()[observation0.id()].t_map_marker().tf();
        auto t_m0_w = t_w_m0.inverse();

        if (add_t_m0_c) {
          const auto &t_w_c = marker_observations.t_map_camera();
          auto t_m0_c = t_m0_w * t_w_c;
          data.initial_.insert(t_m0_c_key, t_m0_c.template to<gtsam::Pose3>().compose(delta));
        }
        if (add_t_m0_m1) {
          auto t_w_m1 = runner_.model().targets()[observation1.id()].t_map_marker().tf();
          auto t_m0_m1 = t_m0_w * t_w_m1;
          data.initial_.insert(t_m0_m1_key, t_m0_m1.template to<gtsam::Pose3>().compose(delta));
//          data.initial_.insert(t_m0_m1_key, gtsam::Pose3{});
        }
      }

      return data;
    }

    int optimize_and_test(GraphData &graph_data)
    {
      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
//          params.setVerbosityLM("TERMINATION");
//          params.setVerbosity("TERMINATION");
//      params.setRelativeErrorTol(1e-12);
//      params.setAbsoluteErrorTol(1e-12);
//      params.setMaxIterations(2048);

//          graph.print("graph\n");
//          initial.print("initial\n");

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_data.graph_, graph_data.initial_, params).optimize();
//      std::cout << "initial error = " << graph_data.graph_.error(graph_data.initial_) << std::endl;
//      std::cout << "final error = " << graph_data.graph_.error(result) << std::endl;
//      result.print("");

      // Clear out the graph for the next optimization
      graph_data.graph_.resize(0);
      graph_data.initial_.clear();

      for (auto f: result.filter<gtsam::Pose3>(
        gtsam::Symbol::ChrTest(fvlam::ModelKey::key_char_marker_marker))) {
        auto t_m0_m1_key = f.key;
        auto &t_m0_m1_actual = f.value;

        auto id0 = fvlam::ModelKey::id0_from_marker_marker(t_m0_m1_key);
        auto id1 = fvlam::ModelKey::id1_from_marker_marker(t_m0_m1_key);

//        runner_.logger().warn() << id0 << " " << id1 << " "
//                                << fvlam::Transform3::from(t_m0_m1_actual).to_string();

        auto t_w_m0 = runner_.model().targets()[id0].t_map_marker().tf();
        auto t_w_m1 = runner_.model().targets()[id1].t_map_marker().tf();
        auto t_m0_w = t_w_m0.inverse();
        auto t_m0_m1_expected = t_m0_w * t_w_m1;

        RETURN_ONE_IF_FALSE(
          runner_.logger(), "InterMarkerPoseTest optimize_and_test",
          gtsam::assert_equal(t_m0_m1_expected.to<gtsam::Pose3>(),
                              t_m0_m1_actual,
                              runner_.cfg().equals_tolerance_));
      }
      return 0;
    }

    int optimize_and_test_all(std::map<std::uint64_t, GraphData> &graph_data_map)
    {
      for (auto &gdp: graph_data_map) {
        RETURN_IF_NONZERO(
          runner_.logger(), "InterMarkerPoseTest optimize_and_test_all",
          optimize_and_test(gdp.second));
      }
      return 0;
    }

    int all_camera(int algorithm)
    {
      int pair_count = 0;
      int optimizations_count = 0;
      std::map<std::uint64_t, GraphData> graph_data_map;

      for (auto famos = runner_.make_for_all_marker_observations(false);
           famos.test(); famos.next()) {
        for (auto faos = fvlam::MarkerModelRunner::ForAllObservations(famos.marker_observations(),
                                                                      runner_.model().camera_info_map());
             faos.test(); faos.next()) {
          for (auto faop = fvlam::MarkerModelRunner::ForAllObservationPair(faos.observations());
               faop.test(); faop.next()) {
            pair_count += 1;

            add_marker_pair_to_graph<GraphData>(
              famos.marker_observations(),
              faos.camera_info(),
              faop.observation0(),
              faop.observation1(),
              graph_data_map,
              [](std::uint64_t key) -> GraphData
              { return GraphData{key}; });
          }

          if (algorithm == 0) {
            optimizations_count += 1;
            RETURN_IF_NONZERO(
              runner_.logger(), "InterMarkerPoseTest observations",
              optimize_and_test_all(graph_data_map));
          }
        }

        if (algorithm == 1) {
          optimizations_count += 1;
          RETURN_IF_NONZERO(
            runner_.logger(), "InterMarkerPoseTest observations",
            optimize_and_test_all(graph_data_map));
        }
      }

      if (algorithm == 2) {
        optimizations_count += 1;
        RETURN_IF_NONZERO(
          runner_.logger(), "InterMarkerPoseTest observations",
          optimize_and_test_all(graph_data_map));
      }

      runner_.logger().warn() << "InterMarkerPoseTest optimizations: " << optimizations_count
                              << " pairs: " << pair_count;
      return 0;
    }

    void fixed_lag_update(std::map<std::uint64_t, FixedLagData> &fixed_lag_data_map)
    {
      for (auto &fldmp: fixed_lag_data_map) {
        auto &data = fldmp.second;

        // Only do an update if a new factor has been added to this graph.
        if (!data.graph_.empty()) {

          // Update the Fixed Lag Smoother for this pair of markers
          data.batch_smoother_.update(data.graph_, data.initial_, data.timestamps_);

          // Update the timestamp for the next set of measurements
          data.current_timestamp_ += 1.0;

          auto t_m0_m1_key = data.t_m0_m1_key_;
          auto id0 = fvlam::ModelKey::id0_from_marker_marker(t_m0_m1_key);
          auto id1 = fvlam::ModelKey::id1_from_marker_marker(t_m0_m1_key);
          runner_.logger().warn() << "Just updated " << std::setw(2) << id0 << " " << std::setw(2) << id1;
//          data.batch_smoother_.getFactors().print("Factors\n");
//          data.batch_smoother_.getDelta().print("Delta\n");

          // Clear containers for the next iteration
          data.graph_.resize(0);
          data.initial_.clear();
          data.timestamps_.clear();
        }
      }
    }

    int fixed_lag_test(std::map<std::uint64_t, FixedLagData> &fixed_lag_data_map)
    {
      for (auto &fldmp: fixed_lag_data_map) {
        auto &data = fldmp.second;

        auto t_m0_m1_key = data.t_m0_m1_key_;
        auto id0 = fvlam::ModelKey::id0_from_marker_marker(t_m0_m1_key);
        auto id1 = fvlam::ModelKey::id1_from_marker_marker(t_m0_m1_key);

        auto t_w_m0 = runner_.model().targets()[id0].t_map_marker().tf();
        auto t_w_m1 = runner_.model().targets()[id1].t_map_marker().tf();
        auto t_m0_w = t_w_m0.inverse();
        auto t_m0_m1_expected = t_m0_w * t_w_m1;

        auto t_m0_m1_actual = data.batch_smoother_.calculateEstimate<gtsam::Pose3>(t_m0_m1_key);

        runner_.logger().warn() << std::setw(2) << id0 << " " << std::setw(2) << id1 << " "
                                << t_m0_m1_expected.to_string() << " "
                                << fvlam::Transform3::from(t_m0_m1_actual).to_string();

        RETURN_ONE_IF_FALSE(
          runner_.logger(), "InterMarkerPoseTest fixed_lag_test",
          gtsam::assert_equal(t_m0_m1_expected.to<gtsam::Pose3>(),
                              t_m0_m1_actual,
                              runner_.cfg().equals_tolerance_));
      }
      return 0;
    }

    int fixed_lag()
    {
      std::map<std::uint64_t, FixedLagData> fixed_lag_data_map;

      // Define the smoother lag in camera indices
      double lag = 3.0;

      // Create a fixed lag smoother
      // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
//      params.setRelativeErrorTol(1e-12);
//      params.setAbsoluteErrorTol(1e-12);
//      params.setMaxIterations(2048);
//      params.lambdaUpperBound = 1e10;


      for (auto famos = runner_.make_for_all_marker_observations(false);
           famos.test(); famos.next()) {

        for (auto faos = fvlam::MarkerModelRunner::ForAllObservations(famos.marker_observations(),
                                                                      runner_.model().camera_info_map());
             faos.test(); faos.next()) {
          for (auto faop = fvlam::MarkerModelRunner::ForAllObservationPair(faos.observations());
               faop.test(); faop.next()) {

//            if (faop.observation0().id() == 10 && faop.observation1().id() == 11) {

              add_marker_pair_to_graph<FixedLagData>(
                famos.marker_observations(),
                faos.camera_info(),
                faop.observation0(),
                faop.observation1(),
                fixed_lag_data_map,
                [&lag, &params](std::uint64_t key) -> FixedLagData
                { return FixedLagData{key, lag, params}; });
//            }
          }
        }

        runner_.logger().warn() << "Do update for camera: " << famos.marker_observations().camera_index();

        fixed_lag_update(fixed_lag_data_map);
      }

      return fixed_lag_test(fixed_lag_data_map);
    }

  public:
    InterMarkerPoseTest(Config cfg, fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner},
      measurement_noise_{gtsam::noiseModel::Isotropic::Sigma(16, 1.0)},
      corners_f_marker_{fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, 4>>(
        runner_.model().environment().marker_length())}
    {}

    int operator()()
    {
      switch (cfg_.algorithm_) {
        default:
        case 0:
        case 1:
        case 2:
          return all_camera(cfg_.algorithm_);

        case 3:
          return fixed_lag();
      }
    }
  };


  int inter_marker_pose_test()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    fvlam::LoggerCout logger{runner_config.logger_level_};

//    auto model_maker = fvlam::MarkerModelGen::MonoParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleCamera();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleMarker();
    auto model_maker = fvlam::MarkerModelGen::MonoSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::DualSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::MonoParallelCircles();
//    auto model_maker = fvlam::MarkerModelGen::MonoDoubleMarker();
//    auto model_maker = fvlam::MarkerModelGen::MonoSingleMarker();
//    auto model_maker = fvlam::MarkerModelGen::DualSingleMarker();

    InterMarkerPoseTest::Config test_config{};

    test_config.algorithm_ = 0;
    runner_config.equals_tolerance_ = 2.e-3;
    RETURN_IF_NONZERO(
      logger, "InterMarkerPoseTest algorithm=" << test_config.algorithm_,
      fvlam::MarkerModelRunner::runner_run<InterMarkerPoseTest>(runner_config, model_maker, test_config));

    test_config.algorithm_ = 1;
    runner_config.equals_tolerance_ = 2.e-3;
    RETURN_IF_NONZERO(
      logger, "Run InterMarkerPoseTest algorithm= " << test_config.algorithm_,
      fvlam::MarkerModelRunner::runner_run<InterMarkerPoseTest>(runner_config, model_maker, test_config));

    test_config.algorithm_ = 2;
    runner_config.equals_tolerance_ = 2.e-3;
    RETURN_IF_NONZERO(
      logger, "Run InterMarkerPoseTest algorithm= " << test_config.algorithm_,
      fvlam::MarkerModelRunner::runner_run<InterMarkerPoseTest>(runner_config, model_maker, test_config));

    test_config.algorithm_ = 3;
    runner_config.equals_tolerance_ = 2.e-3;
    RETURN_IF_NONZERO(
      logger, "Run InterMarkerPoseTest algorithm= " << test_config.algorithm_,
      fvlam::MarkerModelRunner::runner_run<InterMarkerPoseTest>(runner_config, model_maker, test_config));

    return 0;
  }
}

