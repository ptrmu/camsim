
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/model.hpp"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"

namespace camsim
{

  class ProjectBetweenFactorTest
  {
  public:
    using This = ProjectBetweenFactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    ProjectBetweenFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      const gtsam::Pose3 &camera_f_world,
      const gtsam::Pose3 &marker_f_world,
      const gtsam::Point2 &corner_f_image,
      const gtsam::Point3 &corner_f_marker,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = fvlam::ProjectBetweenFactor(corner_f_image, measurement_noise,
                                                0, 1,
                                                corner_f_marker, cal3ds2,
                                                runner_.logger());

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_marker;
      gtsam::Matrix d_point2_wrt_camera;
      factor.evaluateError(marker_f_world,
                           camera_f_world,
                           d_point2_wrt_marker,
                           d_point2_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
        {
          return factor.evaluateError(marker_pose, camera_pose);
        }, marker_f_world, camera_f_world);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
        {
          return factor.evaluateError(marker_pose, camera_pose);
        }, marker_f_world, camera_f_world);

      // Test that the analytic and numerical solutions are the same.
      return (gtsam::assert_equal(numericalH1, d_point2_wrt_marker, 1.0e-6) &&
              gtsam::assert_equal(numericalH2, d_point2_wrt_camera, 1.0e-6)) ? 0 : 1;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      // For each camera location
      for (auto &marker_observations : runner_.marker_observations_list_perturbed()) {
        auto camera_f_world = marker_observations.t_map_camera().to<gtsam::Pose3>();

        // For each imager's observations
        for (auto &observations : marker_observations.observations_synced().v()) {

          // Find the camera_info
          const auto &kp = runner_.model().camera_info_map().m().find(observations.imager_frame_id());
          if (kp == runner_.model().camera_info_map().m().end()) {
            continue;
          }
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(kp->second.to<gtsam::Cal3DS2>());

          // For each observation of a marker
          for (auto &observation : observations.v()) {
            auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
            auto corners_f_image = observation.corners_f_image();
            for (std::size_t i = 0; i < corners_f_marker.size(); i += 1) {

              if (0 != compare_analytic_to_numerical(
                camera_f_world, marker_f_world,
                corners_f_image[i].to<gtsam::Point2>(), corners_f_marker[i],
                cal3ds2, measurement_noise)) {
                runner_.logger().warn() << "Numerical test failed";
                return 1;
              }
            }
          }
        }
      }
//
//      // Create the factor
//      auto factor = fvlam::ProjectBetweenFactor(corners_f_image[0].to<gtsam::Point2>(), measurement_noise,
//                                                0, 1,
//                                                corners_f_marker[0], cal3ds2_ptr,
//                                                runner_.logger());
//
//      // Calculate the Jacobean from the factor
//      gtsam::Matrix d_point2_wrt_marker;
//      gtsam::Matrix d_point2_wrt_camera;
//      factor.evaluateError(marker_pose.to<gtsam::Pose3>(),
//                           camera_pose.to<gtsam::Pose3>(),
//                           d_point2_wrt_marker,
//                           d_point2_wrt_camera);
//
//      // Calculate the Jacobean numerically
//      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
//        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
//        {
//          return factor.evaluateError(marker_pose, camera_pose);
//        }, marker_pose.to<gtsam::Pose3>(), camera_pose.to<gtsam::Pose3>());
//      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
//        [&factor](gtsam::Pose3 marker_pose, gtsam::Pose3 camera_pose) -> gtsam::Point2
//        {
//          return factor.evaluateError(marker_pose, camera_pose);
//        }, marker_pose.to<gtsam::Pose3>(), camera_pose.to<gtsam::Pose3>());
//

      return 0;
    }

  };

  template<class TestMaker>
  int runner_run(const fvlam::MarkerModelRunner::Config &runner_config)
  {
    auto marker_runner = fvlam::MarkerModelRunner(runner_config,
//                                                  fvlam::MarkerModelGen::MonoParallelGrid());
//                                                    fvlam::MarkerModelGen::DualParallelGrid());
//                                                    fvlam::MarkerModelGen::DualWideSingleCamera());
//                                                    fvlam::MarkerModelGen::DualWideSingleMarker());
//                                                  fvlam::MarkerModelGen::MonoSpinCameraAtOrigin());
                                                  fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::MonoParallelCircles());

    auto test_maker = [](fvlam::MarkerModelRunner &runner) -> TestMaker
    {
      return TestMaker(runner);
    };

    auto ret0 = marker_runner.run<typename TestMaker::Maker>(test_maker);

#ifdef ENABLE_TIMING
    gtsam::tictoc_print2_();
#endif
    gtsam::tictoc_reset_();

    return ret0;
  }

  int factors_gtsam_test()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    fvlam::LoggerCout logger{runner_config.logger_level_};


    bool ret = 0;

    runner_config.u_sampler_sigma_ = 1.e-1;
    ret = runner_run<ProjectBetweenFactorTest>(runner_config);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: ProjectBetweenFactorTest ret=" << ret;
      return ret;
    }

#if 0
    runner_config.u_sampler_sigma_ = 1.e-3;
    runner_config.logger_level_ = fvlam::Logger::Levels::level_warn;
    iip_config.algorithm_ = 1;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }
#endif

#if 0
    runner_config.u_sampler_sigma_ = 1.e-3;
    runner_config.logger_level_ = fvlam::Logger::Levels::level_warn;
    iip_config.algorithm_ = 2;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }
#endif

#if 0
    runner_config.u_sampler_sigma_ = 1.e-3;
    runner_config.logger_level_ = fvlam::Logger::Levels::level_warn;
    iip_config.algorithm_ = 3;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }
#endif

#if 0
    runner_config.u_sampler_sigma_ = 1.e-3;
    runner_config.logger_level_ = fvlam::Logger::Levels::level_warn;
    iip_config.algorithm_ = 4;
    ret = runner_run();
    if (ret != 0) {
      logger.warn() << "algorithm_ " << iip_config.algorithm_ << " ret=" << ret;
      return ret;
    }
#endif

    return ret;
  }

}
