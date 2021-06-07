
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/model.hpp"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "cal_info.hpp"

namespace camsim
{
// ==============================================================================
// ProjectBetweenFactorTest class
// ==============================================================================

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
      const gtsam::Pose3 &imager_f_world,
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
                           imager_f_world,
                           d_point2_wrt_marker,
                           d_point2_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_f_world, gtsam::Pose3 imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(marker_f_world, imager_f_world);
        }, marker_f_world, imager_f_world);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 marker_f_world, gtsam::Pose3 imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(marker_f_world, imager_f_world);
        }, marker_f_world, imager_f_world);

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
          auto imager_f_world = (marker_observations.t_map_camera()*camera_info.t_camera_imager()).to<gtsam::Pose3>();
          auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

          return compare_analytic_to_numerical(
            imager_f_world, marker_f_world,
            corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
            cal3ds2, measurement_noise);
        });
    }
  };

// ==============================================================================
// ResectioningFactorTest class
// ==============================================================================

  class ResectioningFactorTest
  {
  public:
    using This = ResectioningFactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    ResectioningFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      const gtsam::Pose3 &imager_f_world,
      const gtsam::Pose3 &marker_f_world,
      const gtsam::Point2 &corner_f_image,
      const gtsam::Point3 &corner_f_world,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = fvlam::ResectioningFactor(0, corner_f_image, measurement_noise,
                                              corner_f_world, cal3ds2, runner_.logger());

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_camera;
      factor.evaluateError(imager_f_world,
                           d_point2_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH = gtsam::numericalDerivative11<gtsam::Point2, gtsam::Pose3>(
        [&factor](gtsam::Pose3 imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(imager_f_world);
        }, imager_f_world);

      // Test that the analytic and numerical solutions are the same.
      return gtsam::assert_equal(numericalH, d_point2_wrt_camera, 1.0e-6) ? 0 : 1;
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
          auto imager_f_world = (marker_observations.t_map_camera()*camera_info.t_camera_imager()).to<gtsam::Pose3>();
          auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

          return compare_analytic_to_numerical(
            imager_f_world, marker_f_world,
            corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
            cal3ds2, measurement_noise);
        });
    }
  };

// ==============================================================================
// QuadResectioningOffsetFactorTest class
// ==============================================================================

  class QuadResectioningOffsetFactorTest
  {
  public:
    using This = QuadResectioningOffsetFactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    QuadResectioningOffsetFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      const gtsam::Pose3 &imager_f_world,
      bool use_transform,
      const gtsam::Pose3 &t_camera_imager,
      const std::vector<gtsam::Point2> &corners_f_image,
      const std::vector<gtsam::Point3> &corners_f_world,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = fvlam::QuadResectioningOffsetFactor(
        0, corners_f_image, measurement_noise,
        corners_f_world, use_transform, t_camera_imager,
        cal3ds2, runner_.logger(), "QuadResectioningOffsetFactorTest");

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2s_wrt_camera;
      factor.evaluateError(imager_f_world,
                           d_point2s_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        [&factor](gtsam::Pose3 imager_f_world) -> gtsam::Vector
        {
          return factor.evaluateError(imager_f_world);
        }, imager_f_world);

      // Test that the analytic and numerical solutions are the same.
      return gtsam::assert_equal(numericalH, d_point2s_wrt_camera, 1.0e-6) ? 0 : 1;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));

      return runner_.for_each_observation(
        [this, &measurement_noise](
          const fvlam::MarkerObservations &marker_observations,
          const fvlam::Observations &observations,
          const fvlam::CameraInfo &camera_info,
          const fvlam::Observation &observation) -> int
        {
          auto imager_f_world = (marker_observations.t_map_camera()*camera_info.t_camera_imager()).to<gtsam::Pose3>();
          auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
          auto &marker = runner_.model().targets()[observation.id()];
          auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
            runner_.model().environment().marker_length());
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());


          return compare_analytic_to_numerical(
            imager_f_world,
            camera_info.t_camera_imager().is_valid(), camera_info.t_camera_imager().to<gtsam::Pose3>(),
            corners_f_image, corners_f_world,
            cal3ds2, measurement_noise);
        });
    }
  };

// ==============================================================================
// MarkerCornerFactorTest class
// ==============================================================================

  class MarkerCornerFactorTest
  {
  public:
    using This = MarkerCornerFactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    MarkerCornerFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      const gtsam::Pose3 &marker_f_world,
      const gtsam::Point3 &corner_f_marker,
      const gtsam::Point3 &corner_f_world,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = MarkerCornerFactor(0, 1,
                                       corner_f_marker, measurement_noise);

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_pose;
      gtsam::Matrix d_point2_wrt_point3;
      factor.evaluateError(marker_f_world,
                           corner_f_world,
                           d_point2_wrt_pose,
                           d_point2_wrt_point3);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
        [&factor](gtsam::Pose3 marker_f_world, gtsam::Point3 corner_f_world) -> gtsam::Point3
        {
          return factor.evaluateError(marker_f_world, corner_f_world);
        }, marker_f_world, corner_f_world);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
        [&factor](gtsam::Pose3 marker_f_world, gtsam::Point3 corner_f_world) -> gtsam::Point3
        {
          return factor.evaluateError(marker_f_world, corner_f_world);
        }, marker_f_world, corner_f_world);

      // Test that the analytic and numerical solutions are the same.
      return (gtsam::assert_equal(numericalH1, d_point2_wrt_pose, 1.0e-6) &&
              gtsam::assert_equal(numericalH2, d_point2_wrt_point3, 1.0e-6)) ? 0 : 1;
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
          auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
          auto &marker = runner_.model().targets()[observation.id()];
          auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
            runner_.model().environment().marker_length());
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

          return compare_analytic_to_numerical(
            marker_f_world,
            corners_f_marker[corner_index],
            corners_f_world[corner_index],
            cal3ds2, measurement_noise);
        });
    }
  };

// ==============================================================================
// Imager0Imager1FactorTest class
// ==============================================================================

  class Imager0Imager1FactorTest
  {
  public:
    using This = Imager0Imager1FactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    Imager0Imager1FactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      const gtsam::Pose3 &t_m_i0,
      const gtsam::Pose3 &t_i0_i1,
      const gtsam::Point2 &corner_f_image,
      const gtsam::Point3 &corner_f_marker,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = Imager0Imager1Factor(0, 1,
                                         corner_f_image, measurement_noise,
                                         corner_f_marker, cal3ds2,
                                         runner_.logger());

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_m_i0_pose;
      gtsam::Matrix d_point2_wrt_i0_i1_pose;
      factor.evaluateError(t_m_i0,
                           t_i0_i1,
                           d_point2_wrt_m_i0_pose,
                           d_point2_wrt_i0_i1_pose);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 t_m_i0, gtsam::Pose3 t_i0_i1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m_i0, t_i0_i1);
        }, t_m_i0, t_i0_i1);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](gtsam::Pose3 t_m_i0, gtsam::Pose3 t_i0_i1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m_i0, t_i0_i1);
        }, t_m_i0, t_i0_i1);

      // Test that the analytic and numerical solutions are the same.
      return (gtsam::assert_equal(numericalH1, d_point2_wrt_m_i0_pose, 1.0e-6) &&
              gtsam::assert_equal(numericalH2, d_point2_wrt_i0_i1_pose, 1.0e-6)) ? 0 : 1;
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
          auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

          // Create a fake scenario with two imagers at various offsets from the first
          for (int i = 0; i < 10; i += 1) {
            auto imager_offset = 0.1 * i + 0.1;
            auto t_i0_i1 = fvlam::Transform3{fvlam::Rotate3{}, fvlam::Translate3{imager_offset, 0, 0}};
            auto camera_f_world = marker_observations.t_map_camera();
            auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf();
            auto t_m_i1 = marker_f_world.inverse() * camera_f_world;
            auto t_m_i0 = t_m_i1 * t_i0_i1.inverse();

            auto ret = compare_analytic_to_numerical(
              t_m_i0.to<gtsam::Pose3>(), t_i0_i1.to<gtsam::Pose3>(),
              corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
              cal3ds2, measurement_noise);
            if (ret != 0) {
              runner_.logger().warn() << "gtsam_factor: MarkerCornerFactorTest compare_analytic_to_numerical ret=" << ret;
              return ret;
            }
          }
          return 0;
        });
    }
  };

  int factors_gtsam_test()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    fvlam::LoggerCout logger{runner_config.logger_level_};

//    auto model_maker = fvlam::MarkerModelGen::MonoParallelGrid();
    auto model_maker = fvlam::MarkerModelGen::DualParallelGrid();
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

    runner_config.u_sampler_sigma_ = 0.;
    ret = fvlam::MarkerModelRunner::runner_run<ResectioningFactorTest>(runner_config, model_maker);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: ResectioningFactorTest ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 0.;
    ret = fvlam::MarkerModelRunner::runner_run<QuadResectioningOffsetFactorTest>(runner_config, model_maker);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: QuadResectioningOffsetFactorTest ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 0.;
    ret = fvlam::MarkerModelRunner::runner_run<MarkerCornerFactorTest>(runner_config, model_maker);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: MarkerCornerFactorTest ret=" << ret;
      return ret;
    }

    runner_config.u_sampler_sigma_ = 0.;
    ret = fvlam::MarkerModelRunner::runner_run<Imager0Imager1FactorTest>(runner_config, model_maker);
    if (ret != 0) {
      logger.warn() << "gtsam_factor: Imager0Imager1FactorTest ret=" << ret;
      return ret;
    }

    return ret;
  }
}
