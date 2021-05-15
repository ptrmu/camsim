#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "smf_run.hpp"
#include "cal_info.hpp"
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>

namespace camsim
{

// ==============================================================================
// ImagerRelativePoseTest class
// ==============================================================================

  class InterImagerPoseTest
  {
  public:
    using This = InterImagerPoseTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

    struct Config
    {
      int algorithm_ = 1; // 0 - single marker, 1 - multiple markers
    };

  private:
    Config cfg_;
    fvlam::MarkerModelRunner &runner_;

    const CalInfo::Map k_map_;
    const std::string base_imager_frame_id_;
    const gtsam::SharedNoiseModel measurement_noise_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    const std::vector<fvlam::Transform3> t_imager0_imagerNs_;

  public:
    InterImagerPoseTest(Config cfg, fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner},
      k_map_{CalInfo::MakeMap(runner_)},
      base_imager_frame_id_{CalInfo::make_base_imager_frame_id(k_map_)},
      measurement_noise_{gtsam::noiseModel::Isotropic::Sigma(2, 1.0)},
      corners_f_marker_{fvlam::Marker::corners_f_marker<std::vector<gtsam::Point3>>(
        runner_.model().environment().marker_length())},
      t_imager0_imagerNs_{CalInfo::make_t_imager0_imagerNs(k_map_)}
    {}

    int single_marker_inter_imager_pose()
    {
      auto camera_0_key = fvlam::ModelKey::camera(0);

      // For each camera.
      for (auto &marker_observations : runner_.marker_observations_list_perturbed()) {

        // for each marker
        for (auto &marker : runner_.model().targets()) {

          // Create a factor graph
          gtsam::NonlinearFactorGraph graph;
          gtsam::Values initial;
          int num_observations = 0;

          // For each imager's observations
          for (auto &observations : marker_observations.observations_synced().v()) {

            // Find the camera_info and K
            const auto &kp = k_map_.find(observations.imager_frame_id());
            if (kp == k_map_.end()) {
              continue;
            }
            auto &cal_info = kp->second;

            // For each observation of a marker
            for (auto &observation : observations.v()) {

              // If this is not the marker we are interested in then continue to search
              if (observation.id() != marker.id()) {
                continue;
              }
              num_observations += 1;

              // For each corner of a marker
              for (std::size_t i = 0; i < observation.corners_f_image().size(); i += 1) {

                if (cal_info.imager_index_ == 0) {

                  // imager 0 just does resection to figure its pose
                  graph.emplace_shared<fvlam::ResectioningFactor>(
                    camera_0_key,
                    observation.corners_f_image()[i].to<gtsam::Point2>(),
                    measurement_noise_,
                    corners_f_marker_[i],
                    cal_info.std_cal3ds2_,
                    runner_.logger(), true);
                } else {

                  // imager n uses a factor relative to imager 0
                  graph.emplace_shared<Imager0Imager1Factor>(
                    camera_0_key, fvlam::ModelKey::camera(cal_info.imager_index_),
                    observation.corners_f_image()[i].to<gtsam::Point2>(),
                    measurement_noise_,
                    corners_f_marker_[i],
                    cal_info.std_cal3ds2_,
                    runner_.logger(), true);
                }
              }
            }

            gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
            if (cal_info.imager_index_ == 0) {

              auto t_marker_camera = marker.t_map_marker().tf().inverse() * marker_observations.t_map_camera();
              initial.insert(camera_0_key, t_marker_camera.to<gtsam::Pose3>().compose(delta));

            } else {

//              initial.insert(fvlam::ModelKey::camera(cal_info.imager_index_),
//                             t_imager0_imagerN[cal_info.imager_index_].to<gtsam::Pose3>().compose(delta));
              initial.insert(fvlam::ModelKey::camera(cal_info.imager_index_), gtsam::Pose3{});
            }
          }

          if (num_observations < 2) {
            continue;
          }

          /* Optimize the graph and print results */
          auto params = gtsam::LevenbergMarquardtParams();
//          params.setVerbosityLM("TERMINATION");
//          params.setVerbosity("TERMINATION");
          params.setRelativeErrorTol(1e-12);
          params.setAbsoluteErrorTol(1e-12);
          params.setMaxIterations(2048);

//          graph.print("graph\n");
//          initial.print("initial\n");

          auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//          std::cout << "initial error = " << graph.error(initial) << std::endl;
//          std::cout << "final error = " << graph.error(result) << std::endl;
//          result.print("");


          for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
            auto t_i0_iN = result.at<gtsam::Pose3>(fvlam::ModelKey::camera(i));
            auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
            if (!t_imager0_imagerNs_[i].equals(t_i0_iN_fvlam, runner_.cfg().equals_tolerance_)) {
              return 1;
            }
          }
        }
      }

      return 0;
    }

    void load_per_camera_inter_imager_factors(const fvlam::MarkerObservations &marker_observations,
                                              gtsam::NonlinearFactorGraph &graph,
                                              gtsam::Values &initial,
                                              gtsam::FixedLagSmoother::KeyTimestampMap *new_timestamps)
    {
      // Create a map of all marker id's that are observed by the base_imager.
      std::map<std::uint64_t, fvlam::Transform3> observed_ids{};
      for (auto &observations : marker_observations.observations_synced().v()) {
        if (observations.imager_frame_id() == base_imager_frame_id_) {
          for (auto &observation : observations.v()) {
            for (auto &marker : runner_.model().targets()) {
              if (marker.id() == observation.id()) {
                observed_ids.emplace(marker.id(), marker.t_map_marker().tf());
                break;
              }
            }
          }
          break;
        }
      }
      if (observed_ids.empty()) {
        return;
      }

      // For each imager's observations
      for (auto &observations : marker_observations.observations_synced().v()) {

        // Find the camera_info and K
        const auto &kp = k_map_.find(observations.imager_frame_id());
        if (kp == k_map_.end()) {
          continue;
        }
        auto &cal_info = kp->second;

        // For each observation of a marker
        for (auto &observation : observations.v()) {

          // If this marker is not seen by the base imager, then don't add its factor.
          // If a marker is only seen by the base imager and not by others, its factor
          // will get added to the graph. This is OK - inefficient but the optimization
          // will not fail
          auto observed_marker = observed_ids.find(observation.id());
          if (observed_marker == observed_ids.end()) {
            continue;
          }
          auto &t_map_marker = observed_marker->second;

          auto camera_n_key = fvlam::ModelKey::camera_marker(
            marker_observations.camera_index(), observation.id());

          // For each corner of a marker
          for (std::size_t i = 0; i < observation.corners_f_image().size(); i += 1) {

            if (cal_info.imager_index_ == 0) {

              // imager 0 just does resection to figure its pose
              graph.emplace_shared<fvlam::ResectioningFactor>(
                camera_n_key,
                observation.corners_f_image()[i].to<gtsam::Point2>(),
                measurement_noise_,
                corners_f_marker_[i],
                cal_info.std_cal3ds2_,
                runner_.logger(), true);

              if (!initial.exists(camera_n_key)) {
                auto t_marker_camera = t_map_marker.inverse() * marker_observations.t_map_camera();
                initial.insert(camera_n_key, t_marker_camera.to<gtsam::Pose3>().compose(
                  gtsam::Pose3(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20))));
              }

            } else {

              auto relative_imager_key = fvlam::ModelKey::value(cal_info.imager_index_);

              // imager n uses a factor relative to imager 0
              graph.emplace_shared<Imager0Imager1Factor>(
                camera_n_key, relative_imager_key,
                observation.corners_f_image()[i].to<gtsam::Point2>(),
                measurement_noise_,
                corners_f_marker_[i],
                cal_info.std_cal3ds2_,
                runner_.logger(), true);

              if (!initial.exists(relative_imager_key)) {
                initial.insert(relative_imager_key, gtsam::Pose3{});
              }
            }
          }
        }
      }
    }

    int per_camera_inter_imager_pose(const fvlam::MarkerObservations &marker_observations)
    {
      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      load_per_camera_inter_imager_factors(
        marker_observations,
        graph, initial, nullptr);

      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
//      params.setVerbosityLM("TERMINATION");
//      params.setVerbosity("TERMINATION");
//      params.setRelativeErrorTol(1e-12);
//      params.setAbsoluteErrorTol(1e-12);
//      params.setMaxIterations(2048);

//      graph.print("graph\n");
//      initial.print("initial\n");

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;
//      result.print("");


      for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
        auto t_i0_iN = result.at<gtsam::Pose3>(fvlam::ModelKey::value(i));
        auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
        if (!t_imager0_imagerNs_[i].equals(t_i0_iN_fvlam, runner_.cfg().equals_tolerance_)) {
          return 1;
        }
      }
      return 0;
    }

    int multi_marker_inter_imager_pose()
    {
      // For each camera.
      for (auto &marker_observations : runner_.marker_observations_list_perturbed()) {
        auto ret = per_camera_inter_imager_pose(marker_observations);
        if (ret != 0) {
          return ret;
        }
      }

      return 0;
    }

    int multi_camera_marker_inter_imager_pose()
    {
      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

//      for (auto &marker_observations : runner_.model().target_observations_list()) {
      for (auto &marker_observations : runner_.marker_observations_list_perturbed()) {
        load_per_camera_inter_imager_factors(
          marker_observations,
          graph, initial, nullptr);
      }

      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
//      params.setVerbosityLM("TERMINATION");
//      params.setVerbosity("TERMINATION");
//      params.setRelativeErrorTol(1e-12);
//      params.setAbsoluteErrorTol(1e-12);
//      params.setMaxIterations(2048);

//      graph.print("graph\n");
//      initial.print("initial\n");

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;
//      result.print("");


      for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
        auto t_i0_iN = result.at<gtsam::Pose3>(fvlam::ModelKey::value(i));
        auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
        if (!t_imager0_imagerNs_[i].equals(t_i0_iN_fvlam, runner_.cfg().equals_tolerance_)) {
          return 1;
        }
      }
      return 0;
    }

    int fixed_lag_inter_imager_pose()
    {

      // Define the smoother lag (in seconds)
      double lag = 2.0;

      // Create a fixed lag smoother
      // The Batch version uses Levenberg-Marquardt to perform the nonlinear optimization
      gtsam::BatchFixedLagSmoother smootherBatch(lag);

      // Create containers to store the factors and linearization points that
      // will be sent to the smoothers
      gtsam::NonlinearFactorGraph newFactors;
      gtsam::Values newValues;
      gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;

      // Add the relative imager pose initial values. Set the time as greater than the number of camera positions.
      for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
        auto t_i0_iN_key = fvlam::ModelKey::value(i);
        newValues.insert(t_i0_iN_key, gtsam::Pose3{});
        newTimestamps[t_i0_iN_key] = runner_.marker_observations_list_perturbed().size();
      }

      for (std::size_t i_camera = 0; i_camera < runner_.marker_observations_list_perturbed().size(); i_camera += 1) {

        // Add the measurements

        // Update and printer the current value
        if (i_camera >= 1) {
          smootherBatch.update(newFactors, newValues, newTimestamps);

          // Print the optimized current pose
          runner_.logger().info() << std::setprecision(5) << "Timestamp = " << i_camera;
          for (std::size_t i = 1; i < t_imager0_imagerNs_.size(); i += 1) {
            auto t_i0_iN = smootherBatch.calculateEstimate<gtsam::Pose3>(fvlam::ModelKey::value(i));
            auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
            runner_.logger().info() << "t_i0_i" << i << ": " << t_i0_iN_fvlam.to_string();
          }

          // Clear containers for the next iteration
          newTimestamps.clear();
          newValues.clear();
          newFactors.resize(0);
        }
      }

      return 1;
    }

    int operator()()
    {
      switch (cfg_.algorithm_) {
        default:
        case 0:
          return single_marker_inter_imager_pose();

        case 1:
          return multi_marker_inter_imager_pose();

        case 2:
          return multi_camera_marker_inter_imager_pose();

        case 3:
          return fixed_lag_inter_imager_pose();
      }
    }
  };

  int imager_relative_pose()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto iip_config = InterImagerPoseTest::Config();

    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto runner_run = [&runner_config, &iip_config]() -> int
    {
      auto marker_runner = fvlam::MarkerModelRunner(runner_config,
//                                                  fvlam::MarkerModelGen::MonoParallelGrid());
                                                  fvlam::MarkerModelGen::DualParallelGrid());
//                                                  fvlam::MarkerModelGen::MonoSpinCameraAtOrigin());
//                                                    fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::MonoParallelCircles());

      auto test_maker = [&iip_config](fvlam::MarkerModelRunner &runner) -> InterImagerPoseTest
      {
        return InterImagerPoseTest(iip_config, runner);
      };

      return marker_runner.run<InterImagerPoseTest::Maker>(test_maker);
    };

    bool ret = 0;

    runner_config.u_sampler_sigma_ = 1.e-5;
    iip_config.algorithm_ = 0;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 1.e-3;
    iip_config.algorithm_ = 1;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 1.e-1;
    iip_config.algorithm_ = 2;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 1.e-7;
    runner_config.logger_level_ = fvlam::Logger::Levels::level_info;
    iip_config.algorithm_ = 3;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }

    return ret;
  }
}
