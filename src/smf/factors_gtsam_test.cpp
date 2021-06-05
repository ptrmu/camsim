
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

      return runner_.for_each_corner_f_image(
        [this, &corners_f_marker, &measurement_noise](
          const fvlam::MarkerObservations &marker_observations,
          const fvlam::Observations &observations,
          const fvlam::CameraInfo &camera_info,
          const fvlam::Observation &observation,
          std::size_t corner_index,
          const fvlam::Translate2 &corner_f_image) -> int
        {
          auto camera_f_world = marker_observations.t_map_camera().to<gtsam::Pose3>();
          auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

          return compare_analytic_to_numerical(
            camera_f_world, marker_f_world,
            corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
            cal3ds2, measurement_noise);
        });
    }
  };

  int factors_gtsam_test()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto model_maker = fvlam::MarkerModelGen::MonoParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleCamera();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleMarker();
//    auto model_maker = fvlam::MarkerModelGen::MonoSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::DualSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::MonoParallelCircles();

    bool ret = 0;

    runner_config.u_sampler_sigma_ = 1.e-1;
    ret = fvlam::MarkerModelRunner::runner_run<ProjectBetweenFactorTest>(runner_config, model_maker);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: ProjectBetweenFactorTest ret=" << ret;
      return ret;
    }

    return ret;
  }

}
