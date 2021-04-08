
#include "smf_run.hpp"

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
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

      // for each camera
      for (auto &marker_observations : runner_.model().target_observations_list()) {

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
                fvlam::ModelKey::camera(marker_observations.camera_index()),
                fvlam::ModelKey::corner(fvlam::ModelKey::marker(observation.id()), i), K);

              // Add an initial location for each corner
            }
          }
        }
      }

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
