
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
  typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3DS2> SmartFactor;

// create a typedef to the camera type
  typedef gtsam::PinholePose<gtsam::Cal3DS2> Camera;


  class SfmSmartFactorTest
  {
  public:
    struct Config
    {
      int sfm_algoriithm_ = 0; // 0 - sfm, 1 - sfm isam, 2 - sfm, smart, 3 sfm, smart, isam
    };

    struct CalInfo
    {
      boost::shared_ptr<gtsam::Cal3DS2> cal3ds2_;
      const fvlam::CameraInfo &camera_info_;

      CalInfo(boost::shared_ptr<gtsam::Cal3DS2> cal3ds2, const fvlam::CameraInfo &camera_info) :
        cal3ds2_{std::move(cal3ds2)}, camera_info_{camera_info}
      {}
    };

    using Cal3DS2Map = std::map<std::string, CalInfo>;

  private:
    const Config cfg_;
    fvlam::MarkerModelRunner &runner_;

    int check_corners(const gtsam::Values &result)
    {
      //Check that the camera poses match the model.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // Get a camera key.
        auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

        // Test the camera pose
        auto t_world_camera = result.at<gtsam::Pose3>(camera_key);
        if (!marker_observations.t_map_camera().equals(fvlam::Transform3::from(t_world_camera))) {
          return 1;
        }
      }

      // Test that the corners match the model.
      for (auto &marker : runner_.model().targets()) {

        auto marker_key = fvlam::ModelKey::marker(marker.id());
        auto corners_f_world = marker.calc_corners3_f_world(
          runner_.model().environment().marker_length());

        // For each corner of a marker
        for (std::size_t i = 0; i < marker.ArraySize; i += 1) {

          auto corner_f_image = result.at<gtsam::Point3>(fvlam::ModelKey::corner(marker_key, i));
          if (!corners_f_world[i].equals(fvlam::Translate3::from<gtsam::Point3>(corner_f_image))) {
            return 1;
          }
        }
      }
      return 0;
    }

    int do_sfm(Cal3DS2Map &k_map,
               gtsam::SharedNoiseModel measurement_noise)
    {

      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      // For each camera.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // Get a camera key.
        auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

        // Add an initial perturbed value for each camera
        gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
        initial.insert(camera_key, marker_observations.t_map_camera().to<gtsam::Pose3>().compose(delta));

        // For each imager's observations
        for (auto &observations : marker_observations.observations_synced().v()) {

          // Find the camera_info and K
          const auto &kp = k_map.find(observations.imager_frame_id());
          if (kp == k_map.end()) {
            continue;
          }

          // For each observation of a marker
          for (auto &observation : observations.v()) {

            // For each corner of a marker
            for (std::size_t i = 0; i < observation.corners_f_image().size(); i += 1) {

              // Maybe the imager is offset from the camera
              auto body_P_sensor =
                kp->second.camera_info_.t_camera_imager().is_valid() ?
                boost::optional<gtsam::Pose3>(kp->second.camera_info_.t_camera_imager().to<gtsam::Pose3>()) :
                boost::none;

              // Add a projection factor for each corner of every marker viewed by an imager
              graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
                observation.corners_f_image()[i].to<gtsam::Point2>(),
                measurement_noise, camera_key,
                fvlam::ModelKey::corner(fvlam::ModelKey::marker(observation.id()), i),
                kp->second.cal3ds2_,
                body_P_sensor);
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

//      graph.print("graph\n");

      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
      std::cout << "initial error = " << graph.error(initial) << std::endl;
      std::cout << "final error = " << graph.error(result) << std::endl;

//      result.print("result\n");

      return check_corners(result);
    }

    int do_sfm_isam(Cal3DS2Map &k_map,
                    gtsam::SharedNoiseModel measurement_noise)
    {
      return 0;
    }

    int do_sfm_smart(Cal3DS2Map &k_map,
                     gtsam::SharedNoiseModel measurement_noise)
    {
#if 0

      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      // For each marker
      for (auto &marker : runner_.model().targets()) {

        // For each corner of that marker
        for (size_t i_corner = 0; i_corner < fvlam::Marker::ArraySize; i_corner += 1) {

          // every landmark represent a single landmark, we use shared pointer to init the factor, and then insert measurements.
          SmartFactor::shared_ptr smartfactor(new SmartFactor(measurement_noise, K));

          // For each camera.
          for (auto &marker_observations : runner_.model().target_observations_list()) {

            // Get a camera key.
            auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

            // Add an initial perturbed value for each camera
            gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
            initial.insert(camera_key, marker_observations.t_map_camera().to<gtsam::Pose3>().compose(delta));

            // For each imager's observations
            for (auto &observations : marker_observations.observations_synced().v()) {

              // Find the camera_info and K
              const auto &kp = k_map.find(observations.imager_frame_id());
              if (kp == k_map.end()) {
                continue;
              }

              // For each observation of a marker
              for (auto &observation : observations.v()) {

                // For each corner of a marker
                for (std::size_t i = 0; i < observation.corners_f_image().size(); i += 1) {

                  // Maybe the imager is offset from the camera
                  auto body_P_sensor =
                    kp->second.camera_info_.t_camera_imager().is_valid() ?
                    boost::optional<gtsam::Pose3>(kp->second.camera_info_.t_camera_imager().to<gtsam::Pose3>()) :
                    boost::none;

                  // Add a projection factor for each corner of every marker viewed by an imager
                  graph.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
                    observation.corners_f_image()[i].to<gtsam::Point2>(),
                    measurement_noise, camera_key,
                    fvlam::ModelKey::corner(fvlam::ModelKey::marker(observation.id()), i),
                    kp->second.cal3ds2_,
                    body_P_sensor);
                }
              }
            }
          }
        }
      }
#endif
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
      // Create a map of calibrations and camera_infos
      auto k_map = Cal3DS2Map{};
      for (auto &cip : runner_.model().camera_info_map().m()) {
        k_map.emplace(cip.first, CalInfo{
          boost::make_shared<gtsam::Cal3DS2>(cip.second.to<gtsam::Cal3DS2>()),
          cip.second});
      }

      // Define the camera observation noise model
      auto measurementNoise =
        gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

      switch (cfg_.sfm_algoriithm_) {
        default:
        case 0:
          return do_sfm(k_map, measurementNoise);
        case 1:
          return do_sfm_isam(k_map, measurementNoise);
        case 2:
          return do_sfm_smart(k_map, measurementNoise);
      }
    }
  };


  int smart_factor_pose_simple(void)
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto smf_test_config = SfmSmartFactorTest::Config();

    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto marker_runner = fvlam::MarkerModelRunner(runner_config,
//                                                  fvlam::MarkerModelGen::MonoParallelGrid());
                                                  fvlam::MarkerModelGen::DualParallelGrid());
//                                                  fvlam::MarkerModelGen::DualSpinCameraAtOrigin());

    auto test_maker = [&smf_test_config](fvlam::MarkerModelRunner &runner) -> SfmSmartFactorTest
    {
      return SfmSmartFactorTest(smf_test_config, runner);
    };

    return marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
  }
}
