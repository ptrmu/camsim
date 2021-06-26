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

    GraphData(gtsam::Key t_m0_m1_key) :
      t_m0_m1_key_{t_m0_m1_key},
      graph_{}, initial_{}
    {}
  };

// ==============================================================================
// MarkerMarkerStorage class
// ==============================================================================

  template<class Stg>
  class MarkerMarkerStorage
  {
    std::map<std::uint64_t, std::map<std::uint64_t, Stg>> storage_{};

  public:
    explicit MarkerMarkerStorage()
    {}

    void clear()
    {
      storage_.clear();
    }

    Stg *lookup(std::uint64_t id0, std::uint64_t id1)
    {
      if (id0 > id1) {
        std::swap(id0, id1);
      }

      auto f_links = storage_.find(id0);
      if (f_links == storage_.end()) {
        return nullptr;
      }

      auto f_link = f_links->second.find(id1);
      if (f_link == f_links->second.end()) {
        return nullptr;
      }

      return &f_link->second;
    }

    Stg &add_or_lookup(std::uint64_t id0, std::uint64_t id1, std::function<Stg(void)> solve_tmm_factory)
    {
      if (id0 > id1) {
        std::swap(id0, id1);
      }

      // Look for thee forward link
      auto f_links = storage_.find(id0);
      if (f_links == storage_.end()) {
        storage_.emplace(id0, std::map<std::uint64_t, Stg>{});
        f_links = storage_.find(id0);
      }

      auto f_link = f_links->second.find(id1);
      if (f_link != f_links->second.end()) {
        return f_link->second;
      }

      // Insert a forward link
      f_links->second.emplace(id1, solve_tmm_factory());
      f_link = f_links->second.find(id1);

      return f_link->second;
    }
  };

  using GraphDataStorage = MarkerMarkerStorage<GraphData>;

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
    std::map<std::uint64_t, GraphData> graph_data_map_;

    const CalInfo::Map k_map_;
    const std::string base_imager_frame_id_;
    const gtsam::SharedNoiseModel measurement_noise_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    std::array<gtsam::Point3, 4> corners_f_marker_array_;
    const std::vector<fvlam::Transform3> t_imager0_imagerNs_;

    GraphData &add_or_lookup(std::uint64_t id0, const std::function<GraphData(void)> &make_graph_data)
    {
      auto pair = graph_data_map_.find(id0);
      if (pair != graph_data_map_.end()) {
        return pair->second;
      }

      graph_data_map_.emplace(id0, make_graph_data());
      return graph_data_map_.find(id0)->second;
    }

    GraphData &add_marker_pair_to_graph(
      const fvlam::MarkerObservations &marker_observations,
      const fvlam::CameraInfo &camera_info,
      const fvlam::Observation &observation0,
      const fvlam::Observation &observation1)
    {
      auto t_m0_c_key = fvlam::ModelKey::camera(marker_observations.camera_index());
      auto t_m0_m1_key = fvlam::ModelKey::marker_marker(observation0.id(), observation1.id());

      auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());
      auto t_camera_imager = camera_info.t_camera_imager().is_valid() ?
                             std::optional<gtsam::Pose3>(camera_info.t_camera_imager().to<gtsam::Pose3>()) :
                             std::nullopt;

      auto m0_corners_f_image = observation0.to<std::array<gtsam::Point2, 4>>();
      auto m1_corners_f_image = observation1.to<std::array<gtsam::Point2, 4>>();

      auto &graph_data = add_or_lookup(t_m0_m1_key,
                                       [t_m0_m1_key]() -> GraphData
                                       { return GraphData(t_m0_m1_key); });

      // Add the factor to the graph
      graph_data.graph_.emplace_shared<QuadMarker0Marker1Factor>(
        t_m0_c_key, t_m0_m1_key,
        m0_corners_f_image,
        m1_corners_f_image,
        measurement_noise_,
        corners_f_marker_array_,
        t_camera_imager,
        cal3ds2, runner_.logger(), "Single Pair Test");

      // Add initial values.
      gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));

      const auto &t_w_c = marker_observations.t_map_camera();
      auto t_w_m0 = runner_.model().targets()[observation0.id()].t_map_marker().tf();
      auto t_w_m1 = runner_.model().targets()[observation1.id()].t_map_marker().tf();

      auto t_m0_w = t_w_m0.inverse();
      auto t_m0_c = t_m0_w * t_w_c;
      auto t_m0_m1 = t_m0_w * t_w_m1;

      if (!graph_data.initial_.exists(t_m0_c_key)) {
        graph_data.initial_.insert(t_m0_c_key, t_m0_c.to<gtsam::Pose3>().compose(delta));
      }
      if (!graph_data.initial_.exists(t_m0_m1_key)) {
        graph_data.initial_.insert(t_m0_m1_key, t_m0_m1.to<gtsam::Pose3>().compose(delta));
      }

      return graph_data;
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

    int optimize_and_test_all()
    {
      for (auto &gdp: graph_data_map_) {
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

      for (auto famos = runner_.make_for_all_marker_observations(false);
           famos.test(); famos.next()) {
        for (auto faos = fvlam::MarkerModelRunner::ForAllObservations(famos.marker_observations(),
                                                                      runner_.model().camera_info_map());
             faos.test(); faos.next()) {
          for (auto faop = fvlam::MarkerModelRunner::ForAllObservationPair(faos.observations());
               faop.test(); faop.next()) {
            pair_count += 1;

            add_marker_pair_to_graph(famos.marker_observations(),
                                     faos.camera_info(),
                                     faop.observation0(),
                                     faop.observation1());
          }

          if (algorithm == 0) {
            optimizations_count += 1;
            RETURN_IF_NONZERO(
              runner_.logger(), "InterMarkerPoseTest observations",
              optimize_and_test_all());
          }
        }

        if (algorithm == 1) {
          optimizations_count += 1;
          RETURN_IF_NONZERO(
            runner_.logger(), "InterMarkerPoseTest observations",
            optimize_and_test_all());
        }
      }

      if (algorithm == 2) {
        optimizations_count += 1;
        RETURN_IF_NONZERO(
          runner_.logger(), "InterMarkerPoseTest observations",
          optimize_and_test_all());
      }

      runner_.logger().warn() << "InterMarkerPoseTest optimizations: " << optimizations_count
                              << " pairs: " << pair_count;
      return 0;
    }

    int fixed_lag_inter_imager_pose()
    {
      if (runner_.logger().output_debug()) {
        gtsam::guardedSetDebug("IncrementalFixedLagSmoother update", true);
//      assert(gtsam::isDebugVersion());
      }

      // Define the smoother lag (in seconds)
      double lag = 3.0;

      // Create a fixed lag smoother
      // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-12);
      params.setAbsoluteErrorTol(1e-12);
      params.setMaxIterations(2048);
      gtsam::BatchFixedLagSmoother smootherBatch(lag, params);

      // The Incremental version uses iSAM2 to perform the nonlinear optimization
      gtsam::ISAM2Params parameters;
      parameters.relinearizeThreshold = 0.0; // Set the relin threshold to zero such that the batch estimate is recovered
      parameters.relinearizeSkip = 1; // Relinearize every time
      gtsam::IncrementalFixedLagSmoother smootherISAM2(lag, parameters);

      // Create containers to store the factors and linearization points that
      // will be sent to the smoothers
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      gtsam::FixedLagSmoother::KeyTimestampMap new_timestamps;

      // Add the relative imager pose initial values. Set the time as greater than the number of camera positions.
      for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
        auto t_i0_iN_key = fvlam::ModelKey::value(i);
        new_values.insert(t_i0_iN_key, t_imager0_imagerNs_[i].to<gtsam::Pose3>());
//        new_timestamps[t_i0_iN_key] = runner_.marker_observations_list_perturbed().size();
      }

      for (std::size_t i_camera = 0; i_camera < runner_.marker_observations_list_perturbed().size(); i_camera += 1) {

        gttic(fixed_lag_inter_imager_pose_loop);

        // Add the measurements
//        load_per_camera_inter_imager_factors(
//          runner_.marker_observations_list_perturbed()[i_camera],
//          new_factors, new_values, &new_timestamps);

        // Update and print the current value
        if (i_camera >= 0) {
          if (runner_.logger().output_debug()) {
            new_factors.print("\nnew factors");
            new_values.print("\nnew values");
          }

          smootherBatch.update(new_factors, new_values, new_timestamps);
//          smootherISAM2.update(new_factors, new_values, new_timestamps);
//          smootherISAM2.update();
//          smootherISAM2.update();

          gttoc(fixed_lag_inter_imager_pose_loop);

          if (runner_.logger().output_debug()) {
            smootherBatch.getFactors().print();
            smootherBatch.getLinearizationPoint().print("Linearization Points\n");
            smootherBatch.getDelta().print("Delta\n");
          }

          // Print the optimized current pose
          runner_.logger().info() << std::setprecision(5) << "Timestamp = " << i_camera;
          for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
            auto t_i0_iN = smootherBatch.calculateEstimate<gtsam::Pose3>(fvlam::ModelKey::value(i));
            auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
//            auto t_i0_iN_isam = smootherISAM2.calculateEstimate<gtsam::Pose3>(fvlam::ModelKey::value(i));
//            auto t_i0_iN_isam_fvlam = fvlam::Transform3::from(t_i0_iN_isam);
            runner_.logger().warn() << i_camera << " batch "
                                    << "t_i0_i" << i << ": "
                                    << t_i0_iN_fvlam.to_string() << " "
              /*<< t_i0_iN_isam_fvlam.to_string()*/;
            if (!t_imager0_imagerNs_[i].equals(t_i0_iN_fvlam, runner_.cfg().equals_tolerance_)) {
              return 1;
            }
          }

#ifdef ENABLE_TIMING
          gtsam::tictoc_print();
          gtsam::tictoc_reset();
#endif

          // Clear containers for the next iteration
          new_timestamps.clear();
          new_values.clear();
          new_factors.resize(0);
        }
      }

      return 0;
    }

  public:
    InterMarkerPoseTest(Config cfg, fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}, graph_data_map_{},
      k_map_{CalInfo::MakeMap(runner_.model().camera_info_map())},
      base_imager_frame_id_{CalInfo::make_base_imager_frame_id(k_map_)},
      measurement_noise_{gtsam::noiseModel::Isotropic::Sigma(16, 1.0)},
      corners_f_marker_{fvlam::Marker::corners_f_marker<std::vector<gtsam::Point3>>(
        runner_.model().environment().marker_length())},
      corners_f_marker_array_{fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, 4>>(
        runner_.model().environment().marker_length())},
      t_imager0_imagerNs_{CalInfo::make_t_imager0_imagerNs(k_map_)}
    {}

    int operator()()
    {
      graph_data_map_.clear();

      switch (cfg_.algorithm_) {
        default:
        case 0:
        case 1:
        case 2:
          return all_camera(cfg_.algorithm_);

        case 3:
          return fixed_lag_inter_imager_pose();
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

//    test_config.algorithm_ = 3;
//    RETURN_IF_NONZERO(
//      logger, "Run InterMarkerPoseTest algorithm= " << test_config.algorithm_,
//      fvlam::MarkerModelRunner::runner_run<InterMarkerPoseTest>(runner_config, model_maker, test_config));

    return 0;
  }
}

