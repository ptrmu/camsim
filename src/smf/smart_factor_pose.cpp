
#include "smf_run.hpp"

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
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
    };

  private:
    const Config cfg_;
    fvlam::MarkerModelRunner &runner_;

  public:
    using Maker = std::function<SfmSmartFactorTest(fvlam::MarkerModelRunner &)>;

    SfmSmartFactorTest(const Config &cfg,
                       fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}
    {}

    bool operator()()
    {
      // Define the camera calibration parameters
      auto K = std::make_shared<gtsam::Cal3_S2>(runner_.model().camera_info_map().first().to<gtsam::Cal3_S2>());

      // Define the camera observation noise model
      auto measurementNoise =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

      return 1;
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
