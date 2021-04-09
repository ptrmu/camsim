
#include "smf_run.hpp"

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

namespace camsim
{


// Make the typename short so it looks much cleaner
  typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

// create a typedef to the camera type
  typedef gtsam::PinholePose<gtsam::Cal3_S2> Camera;


  class SfmSmartFactorTest
  {
  public:
    struct Config
    {
      int sfm_algoriithm_ = 0; // 0 - sfm, 1 - sfm isam, 2 - sfm, smart, 3 sfm, smart, isam
    };

  private:
    const Config cfg_;
    fvlam::MarkerModelRunner &runner_;

    int do_sfm(std::shared_ptr<gtsam::Cal3DS2> K,
               gtsam::SharedNoiseModel measurement_noise)
    {

      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      // For each camera.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // Get a camera key.
        auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

        // Add an initial value for each camera
        initial.insert(camera_key, marker_observations.t_map_camera().to<gtsam::Pose3>());

        // For each imager's observations
        for (auto &observations : marker_observations.observations_synced().v()) {

          // Could create fewer Ks.
          // Find the camera_info and K
          const auto &cip = runner_.model().camera_info_map().m().find(observations.imager_frame_id());
          if (cip == runner_.model().camera_info_map().m().end()) {
            continue;
          }
          const auto K = gtsam::Cal3_S2::shared_ptr(new gtsam::Cal3_S2(cip->second.to<gtsam::Cal3_S2>()));

          // For each observation of a marker
          for (auto &observation : observations.v()) {

            // For each corner of a marker
            for (std::size_t i = 0; i < observation.corners_f_image().size(); i += 1) {

              // Add a projection factor for each corner of every marker viewed by an imager
              graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3_S2>>(
                observation.corners_f_image()[i].to<gtsam::Point2>(),
                measurement_noise,
                camera_key,
                fvlam::ModelKey::corner(fvlam::ModelKey::marker(observation.id()), i), K);
            }
          }
        }
      }

      // Add an initial value for the location of each corner of each marker
      for (auto &marker : runner_.model().targets()) {

        auto marker_key = fvlam::ModelKey::marker(marker.id());
        auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
          runner_.model().environment().marker_length());

        // For each corner of a marker
        for (std::size_t i = 0; i < marker.ArraySize; i += 1) {

          gtsam::Point3 corner_perturbed = corners_f_world[i] + gtsam::Point3(0.1, -0.1, 0.05);
          initial.insert(fvlam::ModelKey::corner(marker_key, i), corner_perturbed);
        }
      }

      // Add a prior for each corner of the first marker.
      auto pointNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
      auto marker0_key = fvlam::ModelKey::marker(runner_.model().targets()[0].id());
      auto corners0_f_world = runner_.model().targets()[0]
        .corners_f_world<std::vector<gtsam::Point3>>(runner_.model().environment().marker_length());
      for (std::size_t i = 0; i < corners0_f_world.size(); i += 1) {

        graph.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
          fvlam::ModelKey::corner(marker0_key, i), corners0_f_world[i], pointNoise);
      }

      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
      std::cout << "initial error = " << graph.error(initial) << std::endl;
      std::cout << "final error = " << graph.error(result) << std::endl;

      result.print("result\n");

      return 0;
    }

  public:
    using Maker = std::function<SfmSmartFactorTest(fvlam::MarkerModelRunner &)>;

    SfmSmartFactorTest(const Config &cfg,
                       fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}
    {}

    bool operator()()
    {
      // Define the camera calibration parameters
      auto K = std::make_shared<gtsam::Cal3DS2>(runner_.model().camera_info_map().first().to<gtsam::Cal3DS2>());

      // Define the camera observation noise model
      auto measurementNoise =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

      switch (cfg_.sfm_algoriithm_) {
        default:
        case 0:
          return do_sfm(K, measurementNoise);
      }
    }
  };


  int smart_factor_pose_simple(void)
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto smf_test_config = SfmSmartFactorTest::Config();

    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto marker_runner = fvlam::MarkerModelRunner(runner_config, fvlam::MarkerModelGen::MonoParallelGrid());

    auto test_maker = [&smf_test_config](fvlam::MarkerModelRunner &runner) -> SfmSmartFactorTest
    {
      return SfmSmartFactorTest(smf_test_config, runner);
    };

    return marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
  }
}
