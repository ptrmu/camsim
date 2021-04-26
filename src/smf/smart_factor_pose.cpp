#pragma ide diagnostic ignored "modernize-use-nodiscard"

#include "smf_run.hpp"

#define ENABLE_TIMING

#include <gtsam/base/timing.h>

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

  // create a typedef to the camera type
  typedef gtsam::PinholePose<gtsam::Cal3DS2> Camera;

  class MarkerCornerFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
  {
    typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> Base;
    typedef MarkerCornerFactor This;

    const gtsam::Point3 corner_f_marker_;

  public:

    MarkerCornerFactor(const gtsam::Key &key_pose, const gtsam::Key &key_point,
                       gtsam::Point3 corner_f_marker,
                       const gtsam::SharedNoiseModel &model) :
      Base(model, key_pose, key_point), corner_f_marker_{std::move(corner_f_marker)}
    {}

    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                const gtsam::Point3 &corner_f_world,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
      if (H2) {
        (*H2) = -1.0 * gtsam::I_3x3;
      }
      return pose.transformFrom(corner_f_marker_, H1) - corner_f_world;
    }

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }
  };

  class Imager0Imager1Factor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    using Base = gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>;
    using This = Imager0Imager1Factor;
    using shared_ptr = boost::shared_ptr<This>;

    gtsam::Key key_t_w_i0_;
    gtsam::Key key_t_i0_i1_;
    gtsam::Point2 point_f_image_;
    gtsam::Point3 point_f_marker_;
    std::shared_ptr<const gtsam::Cal3DS2> cal3ds2_;
    fvlam::Logger &logger_;
    bool throwCheirality_;     // If true, rethrows Cheirality exceptions (default: false)


  public:
    Imager0Imager1Factor(const gtsam::Key &key_t_w_i0, const gtsam::Key &key_t_i0_i1,
                         gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Point3 point_f_marker,
                         std::shared_ptr<const gtsam::Cal3DS2> cal3ds2,
                         fvlam::Logger &logger,
                         bool throwCheirality = false) :
      Base(model, key_t_w_i0, key_t_i0_i1),
      key_t_w_i0_{key_t_w_i0}, key_t_i0_i1_{key_t_i0_i1},
      point_f_image_{point_f_image}, point_f_marker_{point_f_marker},
      cal3ds2_{cal3ds2}, logger_{logger},
      throwCheirality_{throwCheirality}
    {}

    /// @return a deep copy of this factor
    gtsam::NonlinearFactor::shared_ptr clone() const override
    {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
    }


    gtsam::Vector evaluateError(const gtsam::Pose3 &t_w_i0,
                                const gtsam::Pose3 &t_i0_i1,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
      gtsam::Matrix66 d_pose3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;

      // Transform the point from the Marker frame to the World frame
      auto t_w_i1 = t_w_i0.compose(
        t_i0_i1,
        H2 ? gtsam::OptionalJacobian<6, 6>(d_pose3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{t_w_i1, *cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_marker_,
          (H1 || H2) ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3 * d_pose3_wrt_pose3;
        }

        // Return the error.
        return point_f_image - point_f_image_;

      } catch (gtsam::CheiralityException &e) {
        if (H1) *H1 = gtsam::Matrix26::Zero();
        if (H2) *H2 = gtsam::Matrix26::Zero();

        logger_.error() << e.what() << ": t_w_i0 " << gtsam::DefaultKeyFormatter(key_t_w_i0_) <<
                        " moved behind camera " << gtsam::DefaultKeyFormatter(key_t_i0_i1_) << std::endl;

        if (throwCheirality_)
          throw gtsam::CheiralityException(key_t_w_i0_);
      }
      return gtsam::Vector2{2.0 * cal3ds2_->px(), 2.0 * cal3ds2_->py()};
    }
  };


  struct CalInfo
  {
    using Map = std::map<std::string, CalInfo>;

    boost::shared_ptr<gtsam::Cal3DS2> cal3ds2_;
    std::shared_ptr<const gtsam::Cal3DS2> std_cal3ds2_;
    const fvlam::CameraInfo &camera_info_;
    const std::size_t imager_index_;

    CalInfo(boost::shared_ptr<gtsam::Cal3DS2> cal3ds2,
            const fvlam::CameraInfo &camera_info,
            std::size_t imager_index) :
      cal3ds2_{cal3ds2}, std_cal3ds2_{std::make_shared<const gtsam::Cal3DS2>(*cal3ds2)},
      camera_info_{camera_info}, imager_index_{imager_index}
    {}

    static Map MakeMap(const fvlam::MarkerModelRunner &runner)
    {
      auto map = Map{};
      std::size_t imager_index = 0;
      for (auto &cip : runner.model().camera_info_map().m()) {
        auto cal3ds2 = boost::shared_ptr<gtsam::Cal3DS2>(new gtsam::Cal3DS2(cip.second.to<gtsam::Cal3DS2>()));
        map.emplace(cip.first, CalInfo{cal3ds2, cip.second, imager_index++});
      }
      return map;
    }
  };

  class SfmSmartFactorTest
  {
  public:
    struct Config
    {
      int sfm_algoriithm_ = 1; // 0 - sfm, 1 - sfm marker, 2 - sfm, smart, 3 sfm, smart, isam
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
      gtsam::tictoc_print();
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

      return 1;
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

      switch (cfg_.sfm_algoriithm_) {
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

    smf_test_config.sfm_algoriithm_ = 0;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 0 " << ret;
//    if (ret != 0) {
//      return ret;
//    }

    smf_test_config.sfm_algoriithm_ = 1;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 1 " << ret;
//    if (ret != 0) {
//      return ret;
//    }

    smf_test_config.sfm_algoriithm_ = 2;
    ret = marker_runner.run<SfmSmartFactorTest::Maker>(test_maker);
    marker_runner.logger().warn() << "sfm_algoriithm_ 2 " << ret;
    if (ret != 0) {
      return ret;
    }

    return ret;
  }

// ==============================================================================
// ImagerRelativePoseTest class
// ==============================================================================

  class ImagerRelativePoseTest
  {
  public:
    using This = ImagerRelativePoseTest;
    using Maker = std::function<This(fvlam::MarkerModelRunner &)>;

  private:
    fvlam::MarkerModelRunner &runner_;

  public:
    ImagerRelativePoseTest(fvlam::MarkerModelRunner &runner) :
      runner_{runner}
    {}

    int operator()()
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

              initial.insert(fvlam::ModelKey::camera(cal_info.imager_index_),
                             t_imager0_imagerN[cal_info.imager_index_].to<gtsam::Pose3>().compose(delta));
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
  };

  int imager_relative_pose(void)
  {
    auto runner_config = fvlam::MarkerModelRunner::Config();
    auto smf_test_config = SfmSmartFactorTest::Config();

    fvlam::LoggerCout logger{runner_config.logger_level_};

    auto marker_runner = fvlam::MarkerModelRunner(runner_config,
//                                                  fvlam::MarkerModelGen::MonoParallelGrid());
                                                  fvlam::MarkerModelGen::DualParallelGrid());
//                                                  fvlam::MarkerModelGen::MonoSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::DualSpinCameraAtOrigin());
//                                                  fvlam::MarkerModelGen::MonoParallelCircles());

    auto test_maker = [&smf_test_config](fvlam::MarkerModelRunner &runner) -> ImagerRelativePoseTest
    {
      return ImagerRelativePoseTest(runner);
    };

    return marker_runner.run<ImagerRelativePoseTest::Maker>(test_maker);
  }
}
