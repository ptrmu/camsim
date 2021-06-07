#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "smf_run.hpp"

#define ENABLE_TIMING
#include <gtsam/base/timing.h>

#include "cal_info.hpp"
#include "fvlam/factors_gtsam.hpp"
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

  class SfmSmartFactorTest
  {
  public:
    struct Config
    {
      int sfm_algorithm_ = 1; // 0 - sfm, 1 - sfm marker, 2 - sfm, smart, 3 sfm, smart, isam
    };

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
        if (!marker_observations.t_map_camera().equals(fvlam::Transform3::from(t_world_camera),
                                                       runner_.cfg().equals_tolerance_)) {
          return 1;
        }
      }

      // Test that the corners match the model.
      for (auto &marker : runner_.model().targets()) {

        auto marker_key = fvlam::ModelKey::marker(marker.id());
        auto corners_f_world = marker.calc_corners3_f_world(
          runner_.model().environment().marker_length());

        // For each corner of a marker
        for (std::size_t i = 0; i < fvlam::Marker::ArraySize; i += 1) {

          auto corner_f_image = result.at<gtsam::Point3>(fvlam::ModelKey::corner(marker_key, i));
          if (!corners_f_world[i].equals(fvlam::Translate3::from<gtsam::Point3>(corner_f_image),
                                         runner_.cfg().equals_tolerance_)) {
            return 1;
          }
        }
      }
      return 0;
    }

    void do_sfm_graph_emplace(CalInfo::Map &k_map,
                              gtsam::SharedNoiseModel measurement_noise,
                              gtsam::NonlinearFactorGraph &graph)
    {

      // For each camera.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // Get a camera key.
        auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

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
    }

    void do_sfm_initial_insert(CalInfo::Map &k_map,
                               gtsam::Values &initial)
    {
      // For each camera.
      for (auto &marker_observations : runner_.model().target_observations_list()) {

        // Get a camera key.
        auto camera_key = fvlam::ModelKey::camera(marker_observations.camera_index());

        // Add an initial perturbed value for each camera
        gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
        initial.insert(camera_key, marker_observations.t_map_camera().to<gtsam::Pose3>().compose(delta));
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
    }

    auto do_sfm_optimize(gtsam::NonlinearFactorGraph &graph,
                         gtsam::Values &initial)
    {
      /* Optimize the graph and print results */
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-12);
      params.setAbsoluteErrorTol(1e-12);
      params.setMaxIterations(2024);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
      std::cout << "initial error = " << graph.error(initial) << std::endl;
      std::cout << "final error = " << graph.error(result) << std::endl;

//      result.print("result\n");

      return result;
    }

    int do_sfm(CalInfo::Map &k_map,
               gtsam::SharedNoiseModel measurement_noise)
    {
      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      gttic(do_sfm);

      do_sfm_graph_emplace(k_map, measurement_noise, graph);

      do_sfm_initial_insert(k_map, initial);

      auto result = do_sfm_optimize(graph, initial);

      gttoc(do_sfm);
//      gtsam::tictoc_print();
      gtsam::tictoc_reset_();

      return check_corners(result);
    }

    int do_sfm_marker(CalInfo::Map &k_map,
                      gtsam::SharedNoiseModel measurement_noise)
    {
      gttic(do_sfm_marker);

      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      do_sfm_graph_emplace(k_map, measurement_noise, graph);

      do_sfm_initial_insert(k_map, initial);

      // Add marker pose variables and constraints between the corners
      // and the pose of the marker.
      auto corner_location_noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1); //
      for (auto &marker : runner_.model().targets()) {

        auto marker_key = fvlam::ModelKey::marker(marker.id());

        // Add the initial value for the marker pose.
        gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
        initial.insert(marker_key, marker.t_map_marker().tf().to<gtsam::Pose3>().compose(delta));

        // For each corner of a marker add a constraint between the marker pose and the corner location.
        for (std::size_t i = 0; i < marker.ArraySize; i += 1) {
          graph.emplace_shared<MarkerCornerFactor>(
            marker_key, fvlam::ModelKey::corner(marker_key, i),
            marker.corners_f_marker<std::vector<gtsam::Point3>>(runner_.model().environment().marker_length())[i],
            corner_location_noise);
        }
      }

      auto result = do_sfm_optimize(graph, initial);

//      result.print("result\n");

      gttoc(do_sfm_marker);
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();

      return check_corners(result);
    }

    static std::uint64_t make_smart_camera_key(const fvlam::MarkerObservations &marker_observations,
                                               std::size_t imager_index,
                                               std::size_t num_imagers)
    {
      return fvlam::ModelKey::camera(marker_observations.camera_index() * num_imagers + imager_index);
    }

    int do_sfm_smart(CalInfo::Map &k_map,
                     gtsam::SharedNoiseModel measurement_noise)
    {
      // Need at least two cameras
      if (runner_.model().target_observations_list().size() < 2) {
        return 1;
      }

      auto num_imagers = runner_.model().camera_info_map().size();

      gttic(do_sfm_smart);

      // Create a factor graph
      gtsam::NonlinearFactorGraph graph;
      gtsam::Values initial;

      // For each marker
      for (auto &marker : runner_.model().targets()) {

        // For each corner of that marker
        for (size_t i_corner = 0; i_corner < fvlam::Marker::ArraySize; i_corner += 1) {

          // For each imager
          for (auto &cip : k_map) {
            auto &imager_frame_id = cip.first;
            auto &cal_info = cip.second;

            // Maybe the imager is offset from the camera
            auto body_P_sensor =
              cal_info.camera_info_.t_camera_imager().is_valid() ?
              boost::optional<gtsam::Pose3>(cal_info.camera_info_.t_camera_imager().to<gtsam::Pose3>()) :
              boost::none;

            // every landmark represent a single landmark, we use shared pointer to init the factor, and then insert measurements.
            auto smart_factor = boost::make_shared<SmartFactor>(measurement_noise, cal_info.cal3ds2_, body_P_sensor);

            // For each camera.
            for (auto &marker_observations : runner_.model().target_observations_list()) {

              // Find the observations that was made from this imager.
              auto it = marker_observations.observations_synced().v().begin();
              while (it != marker_observations.observations_synced().v().end()) {
                if (it->imager_frame_id() == imager_frame_id) {
                  break;
                }
                ++it;
              }
              if (it == marker_observations.observations_synced().v().end()) {
                continue;
              }
              auto &observations = *it;

              // Find the observation of this marker
              auto o_it = observations.v().begin();
              while (o_it != observations.v().end()) {
                if (o_it->id() == marker.id()) {
                  break;
                }
                ++o_it;
              }
              if (o_it == observations.v().end()) {
                continue;
              }
              auto &observation = *o_it;

              // Get a camera key.
              auto smart_camera_key = make_smart_camera_key(marker_observations, cal_info.imager_index_, num_imagers);

              auto measurement = observation.corners_f_image()[i_corner].to<gtsam::Point2>();
              smart_factor->add(measurement, smart_camera_key);
            }

            // insert the smart factor in the graph
            graph.push_back(smart_factor);
          }
        }
      }

      // Add a prior on pose x0. This indirectly specifies where the origin is.
      // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
      auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << gtsam::Vector3::Constant(0.1), gtsam::Vector3::Constant(0.3)).finished());
      auto &to_list = runner_.model().target_observations_list();
      for (std::size_t i = 0; i < num_imagers; i += 1) {
        graph.addPrior(make_smart_camera_key(to_list[0], i, num_imagers),
                       to_list[0].t_map_camera().to<gtsam::Pose3>(),
                       noise);

        // Because the structure-from-motion problem has a scale ambiguity, the problem is
        // still under-constrained. Here we add a prior on the second pose x1, so this will
        // fix the scale by indicating the distance between x0 and x1.
        // Because these two are fixed, the rest of the poses will be also be fixed.
        graph.addPrior(make_smart_camera_key(to_list[1], i, num_imagers),
                       to_list[1].t_map_camera().to<gtsam::Pose3>(),
                       noise);
      }

//      graph.print("graph\n");

      // Create the initial estimate to the solution
      // Intentionally initialize the variables off from the ground truth
      gtsam::Pose3 delta(gtsam::Rot3::Rodrigues(-0.1, 0.2, 0.25), gtsam::Point3(0.05, -0.10, 0.20));
      for (auto &marker_observations : runner_.model().target_observations_list()) {
        for (size_t i = 0; i < num_imagers; ++i) {
          initial.insert(make_smart_camera_key(marker_observations, i, num_imagers),
                         marker_observations.t_map_camera().to<gtsam::Pose3>().compose(delta));
        }
      }

      // Optimize the graph and print results
      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-12);
      params.setAbsoluteErrorTol(1e-12);

      gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
      auto result = optimizer.optimize();
//      result.print("Final results:\n");

      gttoc(do_sfm_smart);
      gtsam::tictoc_print();
      gtsam::tictoc_reset_();

      return 0; // todo add a test of expected and actual
    }

  public:
    using Maker = std::function<SfmSmartFactorTest(fvlam::MarkerModelRunner &)>;

    SfmSmartFactorTest(const Config &cfg,
                       fvlam::MarkerModelRunner &runner) :
      cfg_{cfg}, runner_{runner}
    {}

    int operator()()
    {
      // Create a map of calibrations and camera_infos
      auto k_map = CalInfo::MakeMap(runner_);

      // Define the camera observation noise model
      auto measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0); // one pixel in u and v

      switch (cfg_.sfm_algorithm_) {
        default:
        case 0:
          return do_sfm(k_map, measurementNoise);
        case 1:
          return do_sfm_marker(k_map, measurementNoise);
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
//                                                  fvlam::MarkerModelGen::DualParallelGrid());
//                                                  fvlam::MarkerModelGen::MonoSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
                                                  fvlam::MarkerModelGen::MonoParallelCircles());

    auto test_maker = [&smf_test_config](fvlam::MarkerModelRunner &runner) -> SfmSmartFactorTest
    {
      return SfmSmartFactorTest(smf_test_config, runner);
    };

    bool ret = 0;

    smf_test_config.sfm_algorithm_ = 0;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 0 " << ret;
//    if (ret != 0) {
//      return ret;
//    }

    smf_test_config.sfm_algorithm_ = 1;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 1 " << ret;
//    if (ret != 0) {
//      return ret;
//    }

    smf_test_config.sfm_algorithm_ = 2;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 2 " << ret;
    if (ret != 0) {
      return ret;
    }

    return ret;
  }
}
