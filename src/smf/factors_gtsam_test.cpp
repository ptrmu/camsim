
#include "fvlam/factors_gtsam.hpp"
#include "fvlam/model.hpp"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/geometry/Pose3.h"
#include "gtsam/geometry/Rot3.h"
#include "cal_info.hpp"

#define RETURN_IF_NONZERO(logger, str, test) \
  do {auto ret = (test); if (ret) { \
  logger.warn() << "factors_gtsam_test " << str << " ret=" << ret; return ret;}} while(false)

#define RETURN_ONE_IF_FALSE(logger, str, test) \
  do {if (!test) { \
  logger.warn() << "factors_gtsam_test " << str; return 1;}} while(false)

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
    explicit ProjectBetweenFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
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
      auto e = factor.evaluateError(marker_f_world,
                                    imager_f_world,
                                    d_point2_wrt_marker,
                                    d_point2_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &marker_f_world, const gtsam::Pose3 &imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(marker_f_world, imager_f_world);
        }, marker_f_world, imager_f_world);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &marker_f_world, const gtsam::Pose3 &imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(marker_f_world, imager_f_world);
        }, marker_f_world, imager_f_world);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "ProjectBetweenFactorTest test_error=true",
          gtsam::assert_equal(gtsam::Point2::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "ProjectBetweenFactorTest d_point2_wrt_marker",
        gtsam::assert_equal(numericalH1, d_point2_wrt_marker, 1.0e-6));
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "ProjectBetweenFactorTest d_point2_wrt_camera",
        gtsam::assert_equal(numericalH2, d_point2_wrt_camera, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "ProjectBetweenFactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_corner_f_image(
            truth_not_perturbed,
            [this, &corners_f_marker, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation,
              std::size_t corner_index,
              const fvlam::Translate2 &corner_f_image) -> int
            {
              auto imager_f_world = (marker_observations.t_map_camera() *
                                     camera_info.t_camera_imager()).to<gtsam::Pose3>();
              auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

              return compare_analytic_to_numerical(
                truth_not_perturbed,
                imager_f_world, marker_f_world,
                corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
                cal3ds2, measurement_noise);
            }));

      } while (truth_not_perturbed);
      return 0;
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
    explicit ResectioningFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
      const gtsam::Pose3 &imager_f_world,
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
      auto e = factor.evaluateError(imager_f_world,
                                    d_point2_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH = gtsam::numericalDerivative11<gtsam::Point2, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &imager_f_world) -> gtsam::Point2
        {
          return factor.evaluateError(imager_f_world);
        }, imager_f_world);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "ResectioningFactorTest test_error=true",
          gtsam::assert_equal(gtsam::Point2::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "ResectioningFactorTest d_point2_wrt_camera",
        gtsam::assert_equal(numericalH, d_point2_wrt_camera, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "ResectioningFactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_corner_f_image(
            truth_not_perturbed,
            [this, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation,
              std::size_t corner_index,
              const fvlam::Translate2 &corner_f_image) -> int
            {
              auto imager_f_world = (marker_observations.t_map_camera() *
                                     camera_info.t_camera_imager()).to<gtsam::Pose3>();
              auto &marker = runner_.model().targets()[observation.id()];
              auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
                runner_.model().environment().marker_length());
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

              return compare_analytic_to_numerical(
                truth_not_perturbed,
                imager_f_world,
                corner_f_image.to<gtsam::Point2>(), corners_f_world[corner_index],
                cal3ds2, measurement_noise);
            }));

      } while (truth_not_perturbed);
      return 0;
    }
  };

// ==============================================================================
// QuadResectioningFactorTest class
// ==============================================================================

  class QuadResectioningFactorTest
  {
  public:
    using This = QuadResectioningFactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    explicit QuadResectioningFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
      const gtsam::Pose3 &camera_f_world,
      bool use_transform,
      const gtsam::Pose3 &t_camera_imager,
      const std::vector<gtsam::Point2> &corners_f_image,
      const std::vector<gtsam::Point3> &corners_f_world,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {

      // Create the factor
      auto factor = fvlam::QuadResectioningFactor(
        0, corners_f_image, measurement_noise,
        corners_f_world, use_transform, t_camera_imager,
        cal3ds2, runner_.logger(), "QuadResectioningFactorTest");

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2s_wrt_camera;
      auto e = factor.evaluateError(camera_f_world,
                                    d_point2s_wrt_camera);

      // Calculate the Jacobean numerically
      auto numericalH = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &imager_f_world) -> gtsam::Vector
        {
          return factor.evaluateError(imager_f_world);
        }, camera_f_world);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "QuadResectioningFactorTest test_error=true",
          gtsam::assert_equal(gtsam::Vector8::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "QuadResectioningFactorTest d_point2_wrt_camera",
        gtsam::assert_equal(numericalH, d_point2s_wrt_camera, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "QuadResectioningFactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_observation(
            truth_not_perturbed,
            [this, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation) -> int
            {
              auto camera_f_world = marker_observations.t_map_camera().to<gtsam::Pose3>();
              auto corners_f_image = observation.to<std::vector<gtsam::Point2>>();
              auto &marker = runner_.model().targets()[observation.id()];
              auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
                runner_.model().environment().marker_length());
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());


              return compare_analytic_to_numerical(
                truth_not_perturbed,
                camera_f_world,
                camera_info.t_camera_imager().is_valid(), camera_info.t_camera_imager().to<gtsam::Pose3>(),
                corners_f_image, corners_f_world,
                cal3ds2, measurement_noise);
            }));

      } while (truth_not_perturbed);
      return 0;
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
    explicit MarkerCornerFactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
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
      auto e = factor.evaluateError(marker_f_world,
                                    corner_f_world,
                                    d_point2_wrt_pose,
                                    d_point2_wrt_point3);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
        [&factor](const gtsam::Pose3 &marker_f_world, const gtsam::Point3 &corner_f_world) -> gtsam::Point3
        {
          return factor.evaluateError(marker_f_world, corner_f_world);
        }, marker_f_world, corner_f_world);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point3, gtsam::Pose3, gtsam::Point3>(
        [&factor](const gtsam::Pose3 &marker_f_world, const gtsam::Point3 &corner_f_world) -> gtsam::Point3
        {
          return factor.evaluateError(marker_f_world, corner_f_world);
        }, marker_f_world, corner_f_world);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "MarkerCornerFactorTest test_error=true",
          gtsam::assert_equal(gtsam::Point3::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "MarkerCornerFactorTest d_point2_wrt_pose",
        gtsam::assert_equal(numericalH1, d_point2_wrt_pose, 1.0e-6));
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "MarkerCornerFactorTest d_point2_wrt_point3",
        gtsam::assert_equal(numericalH2, d_point2_wrt_point3, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "MarkerCornerFactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_observation(
            truth_not_perturbed,
            [this, &corners_f_marker, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation) -> int
            {
              auto marker_f_world = runner_.model().targets()[observation.id()].t_map_marker().tf().to<gtsam::Pose3>();
              auto &marker = runner_.model().targets()[observation.id()];
              auto corners_f_world = marker.corners_f_world<std::vector<gtsam::Point3>>(
                runner_.model().environment().marker_length());
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

              for (std::size_t corner_index = 0; corner_index < fvlam::Marker::ArraySize; corner_index += 1) {
                RETURN_IF_NONZERO(
                  runner_.logger(), "MarkerCornerFactorTest corner_index=" << corner_index,
                  compare_analytic_to_numerical(
                    truth_not_perturbed,
                    marker_f_world,
                    corners_f_marker[corner_index],
                    corners_f_world[corner_index],
                    cal3ds2, measurement_noise));
              };
              return 0;
            }));

      } while (truth_not_perturbed);
      return 0;
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
    explicit Imager0Imager1FactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
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
      auto e = factor.evaluateError(t_m_i0,
                                    t_i0_i1,
                                    d_point2_wrt_m_i0_pose,
                                    d_point2_wrt_i0_i1_pose);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m_i0, const gtsam::Pose3 &t_i0_i1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m_i0, t_i0_i1);
        }, t_m_i0, t_i0_i1, 1e-6);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m_i0, const gtsam::Pose3 &t_i0_i1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m_i0, t_i0_i1);
        }, t_m_i0, t_i0_i1, 1e-6);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "Imager0Imager1FactorTest test_error=true",
          gtsam::assert_equal(gtsam::Point2::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "Imager0Imager1FactorTest d_point2_wrt_m_i0_pose",
        gtsam::assert_equal(numericalH1, d_point2_wrt_m_i0_pose, 1.0e-6));
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "Imager0Imager1FactorTest d_point2_wrt_i0_i1_pose",
        gtsam::assert_equal(numericalH2, d_point2_wrt_i0_i1_pose, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "Imager0Imager1FactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_corner_f_image(
            truth_not_perturbed,
            [this, &corners_f_marker, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation,
              std::size_t corner_index,
              const fvlam::Translate2 &corner_f_image) -> int
            {
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

              // Create a fake scenario with two imagers, the second at various offsets from the first
              for (int i = 0; i < 10; i += 1) {
                auto imager_offset = 0.1 * i + 0.1;
                auto t_i0_i1 = fvlam::Transform3{fvlam::Rotate3{}, fvlam::Translate3{imager_offset, 0, 0}};
                const auto &t_w_c = marker_observations.t_map_camera();
                auto t_w_i1 = camera_info.t_camera_imager().is_valid() ?
                              t_w_c * camera_info.t_camera_imager() :
                              t_w_c;
                auto t_w_m = runner_.model().targets()[observation.id()].t_map_marker().tf();
                auto t_m_i1 = t_w_m.inverse() * t_w_i1;
                auto t_m_i0 = t_m_i1 * t_i0_i1.inverse();

                RETURN_IF_NONZERO(
                  runner_.logger(), "Imager0Imager1FactorTest imager_offset=" << imager_offset,
                  compare_analytic_to_numerical(
                    truth_not_perturbed,
                    t_m_i0.to<gtsam::Pose3>(), t_i0_i1.to<gtsam::Pose3>(),
                    corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
                    cal3ds2, measurement_noise));
              }
              return 0;
            }));

      } while (truth_not_perturbed);
      return 0;
    }
  };

// ==============================================================================
// Marker0Marker1FactorTest class
// ==============================================================================

  class Marker0Marker1FactorTest
  {
  public:
    using This = Marker0Marker1FactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    explicit Marker0Marker1FactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
      const gtsam::Pose3 &t_m0_c,
      const gtsam::Pose3 &t_m0_m1,
      const gtsam::Point2 &corner_f_image,
      const gtsam::Point3 &corner_f_marker,
      const std::optional<gtsam::Pose3> &t_camera_imager,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {
      // Create the factor
      auto factor = Marker0Marker1Factor(0, 1,
                                         corner_f_image, measurement_noise,
                                         corner_f_marker, t_camera_imager,
                                         cal3ds2, runner_.logger());

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_m0_c_pose;
      gtsam::Matrix d_point2_wrt_m0_m1_pose;
      auto e = factor.evaluateError(t_m0_c,
                                    t_m0_m1,
                                    d_point2_wrt_m0_c_pose,
                                    d_point2_wrt_m0_m1_pose);

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m0_c, const gtsam::Pose3 &t_m0_m1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m0_c, t_m0_m1);
        }, t_m0_c, t_m0_m1, 1e-6);
      auto numericalH2 = gtsam::numericalDerivative22<gtsam::Point2, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m0_c, const gtsam::Pose3 &t_m0_m1) -> gtsam::Point2
        {
          return factor.evaluateError(t_m0_c, t_m0_m1);
        }, t_m0_c, t_m0_m1, 1e-6);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "Marker0Marker1FactorTest test_error=true",
          gtsam::assert_equal(gtsam::Point2::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "Marker0Marker1FactorTest d_point2_wrt_m0_c_pose",
        gtsam::assert_equal(numericalH1, d_point2_wrt_m0_c_pose, 1.0e-6));
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "Marker0Marker1FactorTest d_point2_wrt_m0_m1_pose",
        gtsam::assert_equal(numericalH2, d_point2_wrt_m0_m1_pose, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "Marker0Marker1FactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_corner_f_image(
            truth_not_perturbed,
            [this, &corners_f_marker, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info,
              const fvlam::Observation &observation,
              std::size_t corner_index,
              const fvlam::Translate2 &corner_f_image) -> int
            {
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());

              // In the test case, we have the observation (crners_f_image) of the marker by the camera. In the
              // following test scenarios, we create a fake m0 at a specified offset and then pass these values
              // to the custom factor to see if the error and derivatives are correct.
              const auto &t_w_c = marker_observations.t_map_camera();
              auto t_w_m1 = runner_.model().targets()[observation.id()].t_map_marker().tf();
              auto t_m1_c = t_w_m1.inverse() * t_w_c;

              for (int i = 0; i < 10; i += 1) {
                auto marker_offset = 0.2 * i + 0.2;
                auto t_m0_m1 = fvlam::Transform3{fvlam::Rotate3{}, fvlam::Translate3{marker_offset, 0, 0}};
                auto t_m0_c = t_m0_m1 * t_m1_c;

                auto t_camera_imager = camera_info.t_camera_imager().is_valid() ?
                                       std::optional<gtsam::Pose3>(camera_info.t_camera_imager().to<gtsam::Pose3>()) :
                                       std::nullopt;

                RETURN_IF_NONZERO(
                  runner_.logger(), "Marker0Marker1FactorTest imager_offset=" << marker_offset,
                  compare_analytic_to_numerical(
                    truth_not_perturbed,
                    t_m0_c.to<gtsam::Pose3>(), t_m0_m1.to<gtsam::Pose3>(),
                    corner_f_image.to<gtsam::Point2>(), corners_f_marker[corner_index],
                    t_camera_imager,
                    cal3ds2, measurement_noise));
              }
              return 0;
            }));

      } while (truth_not_perturbed);
      return 0;
    }
  };

// ==============================================================================
// QuadMarker0Marker1FactorTest class
// ==============================================================================

  class QuadMarker0Marker1FactorTest
  {
  public:
    using This = QuadMarker0Marker1FactorTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    explicit QuadMarker0Marker1FactorTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}


    int compare_analytic_to_numerical(
      bool test_error,
      const gtsam::Pose3 &t_m0_c,
      const gtsam::Pose3 &t_m0_m1,
      const std::array<gtsam::Point2, 4> &m0_corners_f_image,
      const std::array<gtsam::Point2, 4> &m1_corners_f_image,
      const std::array<gtsam::Point3, 4> &corners_f_marker,
      const std::optional<gtsam::Pose3> &t_camera_imager,
      std::shared_ptr<const gtsam::Cal3DS2> &cal3ds2,
      gtsam::SharedNoiseModel &measurement_noise)
    {
      // Create the factor
      auto factor = QuadMarker0Marker1Factor(0, 1,
                                             m0_corners_f_image, m1_corners_f_image,
                                             measurement_noise,
                                             corners_f_marker, t_camera_imager,
                                             cal3ds2,
                                             runner_.logger());

      // Calculate the Jacobean from the factor
      gtsam::Matrix d_point2_wrt_m0_c_pose;
      gtsam::Matrix d_point2_wrt_m0_m1_pose;
      auto e = factor.evaluateError(t_m0_c,
                                    t_m0_m1,
                                    d_point2_wrt_m0_c_pose,
                                    d_point2_wrt_m0_m1_pose);

      runner_.logger().warn() << e;
      runner_.logger().warn() << d_point2_wrt_m0_c_pose;

      // Calculate the Jacobean numerically
      auto numericalH1 = gtsam::numericalDerivative21<Eigen::Matrix<double, 16, 1>, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m0_c, const gtsam::Pose3 &t_m0_m1) -> Eigen::Matrix<double, 16, 1>
        {
          return factor.evaluateError(t_m0_c, t_m0_m1);
        }, t_m0_c, t_m0_m1, 1e-6);
      auto numericalH2 = gtsam::numericalDerivative22<Eigen::Matrix<double, 16, 1>, gtsam::Pose3, gtsam::Pose3>(
        [&factor](const gtsam::Pose3 &t_m0_c, const gtsam::Pose3 &t_m0_m1) -> Eigen::Matrix<double, 16, 1>
        {
          return factor.evaluateError(t_m0_c, t_m0_m1);
        }, t_m0_c, t_m0_m1, 1e-6);

      if (test_error) {
        RETURN_ONE_IF_FALSE(
          runner_.logger(), "QuadMarker0Marker1FactorTest test_error=true",
          gtsam::assert_equal(Eigen::Matrix<double, 16, 1>::Zero(), e, 1.0e-6));
      }

      // Test that the analytic and numerical derivatives are the same.
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "QuadMarker0Marker1FactorTest d_point2_wrt_m0_c_pose",
        gtsam::assert_equal(numericalH1, d_point2_wrt_m0_c_pose, 1.0e-6));
      RETURN_ONE_IF_FALSE(
        runner_.logger(), "QuadMarker0Marker1FactorTest d_point2_wrt_m0_m1_pose",
        gtsam::assert_equal(numericalH2, d_point2_wrt_m0_m1_pose, 1.0e-6));
      return 0;
    }

    int operator()()
    {
      gtsam::SharedNoiseModel measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
      auto corners_f_marker = fvlam::Marker::corners_f_marker<std::array<gtsam::Point3, fvlam::Marker::ArraySize>>(
        runner_.model().environment().marker_length());

      bool truth_not_perturbed = false;
      do {
        truth_not_perturbed = !truth_not_perturbed;

        RETURN_IF_NONZERO(
          runner_.logger(), "QuadMarker0Marker1FactorTest truth_not_perturbed=" << truth_not_perturbed,
          runner_.for_each_observations(
            truth_not_perturbed,
            [this, &corners_f_marker, &measurement_noise, truth_not_perturbed](
              const fvlam::MarkerObservations &marker_observations,
              const fvlam::Observations &observations,
              const fvlam::CameraInfo &camera_info) -> int
            {
              auto cal3ds2 = std::make_shared<const gtsam::Cal3DS2>(camera_info.to<gtsam::Cal3DS2>());
              const auto &t_w_c = marker_observations.t_map_camera();

              // Loop over all the pairs of observations.
              for (std::size_t i0 = 0; i0 < observations.v().size(); i0 += 1) {
                for (std::size_t i1 = i0 + 1; i1 < observations.v().size(); i1 += 1) {
                  auto m0_corners_f_image = observations.v()[i0].to<std::array<gtsam::Point2, 4>>();
                  auto m1_corners_f_image = observations.v()[i1].to<std::array<gtsam::Point2, 4>>();

                  auto t_w_m0 = runner_.model().targets()[observations.v()[i0].id()].t_map_marker().tf();
                  auto t_w_m1 = runner_.model().targets()[observations.v()[i1].id()].t_map_marker().tf();

                  auto t_m0_w = t_w_m0.inverse();
                  auto t_m0_c = t_m0_w * t_w_c;
                  auto t_m0_m1 = t_m0_w * t_w_m1;

                  auto t_camera_imager = camera_info.t_camera_imager().is_valid() ?
                                         std::optional<gtsam::Pose3>(camera_info.t_camera_imager().to<gtsam::Pose3>()) :
                                         std::nullopt;

                  RETURN_IF_NONZERO(
                    runner_.logger(), "QuadMarker0Marker1FactorTest i0, i1=" << i0 << ", " << i1,
                    compare_analytic_to_numerical(
                      truth_not_perturbed,
                      t_m0_c.to<gtsam::Pose3>(), t_m0_m1.to<gtsam::Pose3>(),
                      m0_corners_f_image, m1_corners_f_image,
                      corners_f_marker,
                      t_camera_imager,
                      cal3ds2, measurement_noise));
                }
              }
              return 0;
            }));

      } while (truth_not_perturbed);
      return 0;
    }
  };

  int factors_gtsam_test()
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    fvlam::LoggerCout logger{runner_config.logger_level_};

//    auto model_maker = fvlam::MarkerModelGen::MonoParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualParallelGrid();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleCamera();
//    auto model_maker = fvlam::MarkerModelGen::DualWideSingleMarker();
//    auto model_maker = fvlam::MarkerModelGen::MonoSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::DualSpinCameraAtOrigin();
//    auto model_maker = fvlam::MarkerModelGen::MonoParallelCircles();
    auto model_maker = fvlam::MarkerModelGen::MonoDoubleMarker();
//    auto model_maker = fvlam::MarkerModelGen::MonoSingleMarker();
//    auto model_maker = fvlam::MarkerModelGen::DualSingleMarker();

//    RETURN_IF_NONZERO(
//      logger, "Run ProjectBetweenFactorTest",
//      fvlam::MarkerModelRunner::runner_run<ProjectBetweenFactorTest>(runner_config, model_maker));
//
//    RETURN_IF_NONZERO(
//      logger, "Run ResectioningFactorTest",
//      fvlam::MarkerModelRunner::runner_run<ResectioningFactorTest>(runner_config, model_maker));
//
//    RETURN_IF_NONZERO(
//      logger, "Run QuadResectioningFactorTest",
//      fvlam::MarkerModelRunner::runner_run<QuadResectioningFactorTest>(runner_config, model_maker));
//
//    RETURN_IF_NONZERO(
//      logger, "Run MarkerCornerFactorTest",
//      fvlam::MarkerModelRunner::runner_run<MarkerCornerFactorTest>(runner_config, model_maker));
//
//    RETURN_IF_NONZERO(
//      logger, "Run Imager0Imager1FactorTest",
//      fvlam::MarkerModelRunner::runner_run<Imager0Imager1FactorTest>(runner_config, model_maker));

    RETURN_IF_NONZERO(
      logger, "Run Marker0Marker1FactorTest",
      fvlam::MarkerModelRunner::runner_run<Marker0Marker1FactorTest>(runner_config, model_maker));

    RETURN_IF_NONZERO(
      logger, "Run QuadMarker0Marker1FactorTest",
      fvlam::MarkerModelRunner::runner_run<QuadMarker0Marker1FactorTest>(runner_config, model_maker));

    return 0;
  }
}
