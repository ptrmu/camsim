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

  public:
    InterImagerPoseTest(Config cfg, fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}
    {}

    int single_marker_inter_imager_pose()
    {
      auto k_map = CalInfo::MakeMap(runner_);

      // Define the camera observation noise model
      auto measurement_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

      auto camera_0_key = fvlam::ModelKey::camera(0);

      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::vector<gtsam::Point3>>(
        runner_.model().environment().marker_length());

      // Find the relative pose of the imagers to the zero'th imager. Used for initial.
      std::vector<fvlam::Transform3> t_imager0_imagerN{k_map.size()};
      for (auto &kp : k_map) {
        t_imager0_imagerN[kp.second.imager_index_] = kp.second.camera_info_.t_camera_imager();
      }
      if (t_imager0_imagerN.size() < 2 || !t_imager0_imagerN[0].is_valid()) {
        return false;
      }
      for (std::size_t i = 1; i < t_imager0_imagerN.size(); i += 1) {
        t_imager0_imagerN[i] = t_imager0_imagerN[0].inverse() * t_imager0_imagerN[i];
      }

      // For each camera.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // for each marker
        for (auto &marker : runner_.model().targets()) {

          // Create a factor graph
          gtsam::NonlinearFactorGraph graph;
          gtsam::Values initial;
          int num_observations = 0;

          // For each imager's observations
          for (auto &observations : marker_observations.observations_synced().v()) {

            // Find the camera_info and K
            const auto &kp = k_map.find(observations.imager_frame_id());
            if (kp == k_map.end()) {
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
                    measurement_noise,
                    corners_f_marker[i],
                    cal_info.std_cal3ds2_,
                    runner_.logger(), true);
                } else {

                  // imager n uses a factor relative to imager 0
                  graph.emplace_shared<Imager0Imager1Factor>(
                    camera_0_key, fvlam::ModelKey::camera(cal_info.imager_index_),
                    observation.corners_f_image()[i].to<gtsam::Point2>(),
                    measurement_noise,
                    corners_f_marker[i],
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


          for (std::size_t i = 1; i < t_imager0_imagerN.size(); i += 1) {
            auto t_i0_iN = result.at<gtsam::Pose3>(fvlam::ModelKey::camera(i));
            auto t_i0_iN_fvlam = fvlam::Transform3::from(t_i0_iN);
            if (!t_imager0_imagerN[i].equals(t_i0_iN_fvlam, runner_.cfg().equals_tolerance_)) {
              return 1;
            }
          }
        }
      }

      return 0;
    }

    void per_camera_inter_imager_pose(const CalInfo::Map &k_map,
                                      const fvlam::MarkerObservations &observations,
                                      gtsam::NonlinearFactorGraph &graph,
                                      gtsam::Values &initial)
    {

    }

    int multi_marker_inter_imager_pose()
    {
      auto k_map = CalInfo::MakeMap(runner_);

      return 0;
    }

    int operator()()
    {
      switch (cfg_.algorithm_) {
        default:
        case 0:
          return single_marker_inter_imager_pose();

        case 1:
          return multi_marker_inter_imager_pose();
      }
    }
  };

  int imager_relative_pose(void)
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto iip_config = InterImagerPoseTest::Config();

    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto marker_runner = fvlam::MarkerModelRunner(runner_config,
//                                                  fvlam::MarkerModelGen::MonoParallelGrid());
                                                  fvlam::MarkerModelGen::DualParallelGrid());
//                                                  fvlam::MarkerModelGen::MonoSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::MonoParallelCircles());

    auto test_maker = [&iip_config](fvlam::MarkerModelRunner &runner) -> InterImagerPoseTest
    {
      return InterImagerPoseTest(iip_config, runner);
    };

    bool ret = 0;

    iip_config.algorithm_ = 0;
    ret = marker_runner.run<InterImagerPoseTest::Maker>(test_maker);
    if (ret != 0) {
      marker_runner.logger().warn() << "algorithm_ 0 " << ret;
      return ret;
    }

    iip_config.algorithm_ = 1;
    ret = marker_runner.run<InterImagerPoseTest::Maker>(test_maker);
    if (ret != 0) {
      marker_runner.logger().warn() << "algorithm_ 1 " << ret;
      return ret;
    }

    return ret;
  }
}
