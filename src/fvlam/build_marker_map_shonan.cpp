
#include <memory>
#include <fvlam/camera_info.hpp>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/marker_map.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  class BuildMarkerMapShonan : public BuildMarkerMapInterface
  {
    const BuildMarkerMapShonanContext context_;
    SolveTMarker0Marker1Factory solve_tmm_factory_;
    const double marker_length_;
    const fvlam::Marker fixed_marker_;

    std::map<std::uint64_t, std::unique_ptr<SolveTMarker0Marker1Interface>> solve_tmm_map_;

    static fvlam::Marker find_fixed_marker(const MarkerMap &map)
    {
      for (auto &marker : map.markers()) {
        if (marker.second.is_fixed()) {
          return marker.second;
        }
      }
      return fvlam::Marker{0, Transform3WithCovariance{}};
    }

    gtsam::ShonanAveraging3::Measurements load_shonan_measurements()
    {
      gtsam::ShonanAveraging3::Measurements measurements{};

      for (auto it = solve_tmm_map_.begin(); it != solve_tmm_map_.end(); ++it) {
        std::uint64_t id0 = it->first / 1000000L;
        std::uint64_t id1 = it->first % 1000000L;

        auto mean = it->second->t_marker0_marker1();

//        std::cout << "id0:" << id0 << " id1:" << id1
//                  << " mean " << mean.to_string(true) << std::endl;

        // shonan noise models must be isotropic?? So take the largest of the diagonal elements.
        auto cov = mean.cov();
        auto var = cov(0, 0);
        if (var < cov(1, 1)) {
          var = cov(1, 1);
        }
        if (var < cov(2, 2)) {
          var = cov(2, 2);
        }
        if (var < 0.0001) {
          var = 0.0001;
        }

        auto measurement = gtsam::BinaryMeasurement<gtsam::Rot3>{
          id0, id1, mean.tf().r().to<gtsam::Rot3>(),
          gtsam::noiseModel::Diagonal::Variances(fvlam::Rotate3::MuVector::Ones() * var)};
        measurements.emplace_back(measurement);
      }

      return measurements;
    }

    gtsam::NonlinearFactorGraph load_pose_graph()
    {
      gtsam::NonlinearFactorGraph pose_graph{};
      auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(Transform3::MuVector::Ones() * 0.1);

      for (auto it = solve_tmm_map_.begin(); it != solve_tmm_map_.end(); ++it) {
        std::uint64_t id0 = it->first / 1000000L;
        std::uint64_t id1 = it->first % 1000000L;

        auto t_marker0_marker1 = it->second->t_marker0_marker1();
        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          id0, id1, t_marker0_marker1.tf().to<gtsam::Pose3>(),
          gtsam::noiseModel::Gaussian::Covariance(t_marker0_marker1.cov()));
      }

      // Add the prior for the fixed node.
      pose_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        fixed_marker_.id(), fixed_marker_.t_world_marker().tf().to<gtsam::Pose3>(),
        gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Pose3::TangentVector::Zero()));

      return pose_graph;
    }

    gtsam::Values load_pose_initial(const gtsam::Values &shonan_result)
    {
      auto r_world_shonan{gtsam::Rot3::identity()};

      // Find the rotation that the shonan algorithm returned for the fixed
      // marker. Then figure out the delta rotation to rotate that shonan
      // rotation to the fixed rotation. Then apply this rotation to all
      // shonan rotations as we are entering them in the initial values.
      for (const auto &key_value : shonan_result) {
        if (key_value.key == fixed_marker_.id()) {
          r_world_shonan = fixed_marker_.t_world_marker().tf().r().to<gtsam::Rot3>() *
                           shonan_result.at<typename gtsam::Pose3::Rotation>(key_value.key).inverse();
          break;
        }
      }

      // Upgrade rotations to full poses
      gtsam::Values pose_initial{};
      for (const auto &key_value : shonan_result) {
        gtsam::Key key = key_value.key;
        const auto &rot = shonan_result.at<typename gtsam::Pose3::Rotation>(key);
        auto rot_f_world = r_world_shonan * rot;
        std::cout << key << " " << fvlam::Rotate3::from(rot_f_world).to_string() << std::endl;
        auto initializedPose = gtsam::Pose3(rot_f_world, gtsam::Pose3::Translation::Zero());
        pose_initial.insert(key, initializedPose);
      }

      return pose_initial;
    }

    std::unique_ptr<MarkerMap> load_map(const gtsam::NonlinearFactorGraph &pose_graph,
                                        const gtsam::Values &pose_result)
    {
      auto map = std::make_unique<MarkerMap>(marker_length_);
      gtsam::Marginals marginals(pose_graph, pose_result);

      for (const auto &key_value : pose_result) {
        gtsam::Key key = key_value.key;
        if (key == fixed_marker_.id()) {
          map->add_marker(fixed_marker_);
        } else {
          const auto &pose = pose_result.at<gtsam::Pose3>(key);
          auto cov = marginals.marginalCovariance(key);

          map->add_marker(Marker{key,
                                 Transform3WithCovariance{Transform3::from(pose), cov}});
        }
      }

      return map;
    }

  public:
    BuildMarkerMapShonan() = delete;

    BuildMarkerMapShonan(BuildMarkerMapShonanContext context,
                         SolveTMarker0Marker1Factory solve_tmm_factory,
                         const MarkerMap &map) :
      context_{std::move(context)},
      solve_tmm_factory_{std::move(solve_tmm_factory)},
      marker_length_{map.marker_length()},
      fixed_marker_{find_fixed_marker(map)},
      solve_tmm_map_{}
    {}

    void process(const MarkerObservations &marker_observations,
                 const CameraInfo &camera_info) override
    {
      auto cv_camera_info = camera_info.to<CvCameraCalibration>();
      auto solve_t_marker0_marker1 = fvlam::Marker::solve_t_marker0_marker1(cv_camera_info, marker_length_);
      auto &obs = marker_observations.observations();

      // Walk through all pairs of observations
      for (std::size_t m0 = 0; m0 < marker_observations.size(); m0 += 1)
        for (std::size_t m1 = m0 + 1; m1 < marker_observations.size(); m1 += 1) {
          std::size_t m0r{m0};
          std::size_t m1r{m1};

          // Alawys do the transform calculation with the lower id first.
          if (obs[m1r].id() < obs[m0r].id()) {
            m0r = m1;
            m1r = m0;
          }

          // Find the appropriate SolveTmmInterface
          std::uint64_t joint_id = obs[m0r].id() * 1000000 + obs[m1r].id();
          auto it = solve_tmm_map_.find(joint_id);
          if (it == solve_tmm_map_.end()) {
            auto res = solve_tmm_map_.emplace(joint_id, solve_tmm_factory_());
            assert(res.second);
            it = res.first;
          }
          it->second->accumulate(obs[m0r], obs[m1r]);
        }
    }

    std::unique_ptr<MarkerMap> build() override
    {
      // Optimize rotations.
      auto shonan_measurements = load_shonan_measurements();
      gtsam::ShonanAveraging3 shonan(shonan_measurements);
      auto shonan_initial = shonan.initializeRandomly();
      auto shonan_result = shonan.run(shonan_initial);
      std::cout << "error of Shonan Averaging " << shonan_result.second << std::endl;
//      shonan_result.first.print("\nshonan_result\n");

      // Prepare for the full Pose optimization
      auto pose_graph = load_pose_graph();
      auto pose_initial = load_pose_initial(shonan_result.first);

//      pose_graph.print("pose_graph\n");
//      pose_initial.print("pose_initial");

      // Do the pose optimization
      gtsam::GaussNewtonParams params;
      params.setVerbosity("TERMINATION");
      gtsam::GaussNewtonOptimizer optimizer(pose_graph, pose_initial, params);
      auto pose_result = optimizer.optimize();

      return load_map(pose_graph, pose_result);
    }

    std::string reset(std::string &cmd) override
    {
      return std::string{};
    }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapShonanContext>(
    const BuildMarkerMapShonanContext &bmm_context,
    SolveTMarker0Marker1Factory solve_tmm_factory,
    const MarkerMap &map_initial)
  {
    return std::make_unique<BuildMarkerMapShonan>(bmm_context, std::move(solve_tmm_factory), map_initial);
  }

// ==============================================================================
// SolveTmmCvSolvePnp class
// ==============================================================================

  class SolveTmmCvSolvePnp : public SolveTMarker0Marker1Interface
  {
    const SolveTmmContextCvSolvePnp solve_tmm_context_;
    fvlam::Marker::SolveFunction solve_t_camera_marker_function_;
    EstimateTransform3MeanAndCovariance emac_algebra_; // Averaging in the vector space
    EstimateMeanAndCovariance<Transform3::MuVector> emac_group_; // Averaging on the manifold

  public:
    SolveTmmCvSolvePnp(const SolveTmmContextCvSolvePnp &solve_tmm_context,
                       const CameraInfo &camera_info,
                       double marker_length) :
      solve_tmm_context_{solve_tmm_context},
      solve_t_camera_marker_function_{fvlam::Marker::solve_t_camera_marker<CvCameraCalibration>
                                        (camera_info.to<CvCameraCalibration>(), marker_length)},
      emac_algebra_{}, emac_group_{}
    {}

    void accumulate(const MarkerObservation &observation0,
                    const MarkerObservation &observation1) override
    {
      auto t_camera_marker0 = solve_t_camera_marker_function_(observation0).t_world_marker().tf();
      auto t_camera_marker1 = solve_t_camera_marker_function_(observation1).t_world_marker().tf();
      auto t_marker0_marker1 = t_camera_marker0.inverse() * t_camera_marker1;
      if (solve_tmm_context_.average_on_space_not_manifold) {
        emac_algebra_.accumulate(t_marker0_marker1);
      } else {
        emac_group_.accumulate(t_marker0_marker1.mu());
      }
    }

    // Given the observations that have been added so far, create and return a marker_map.
    Transform3WithCovariance t_marker0_marker1() override
    {
      return Transform3WithCovariance{
        solve_tmm_context_.average_on_space_not_manifold ? emac_algebra_.mean() : Transform3(emac_group_.mean()),
        solve_tmm_context_.average_on_space_not_manifold ? emac_algebra_.cov() : emac_group_.cov()};
    }
  };

  template<>
  SolveTMarker0Marker1Factory make_solve_tmm_factory<SolveTmmContextCvSolvePnp>
    (const SolveTmmContextCvSolvePnp &solve_tmm_context,
     const CameraInfo &camera_info,
     double marker_length)
  {

    return [
      &solve_tmm_context,
      &camera_info,
      marker_length
    ]() -> std::unique_ptr<SolveTMarker0Marker1Interface>
    {
      return std::make_unique<SolveTmmCvSolvePnp>(solve_tmm_context, camera_info, marker_length);
    };
  }
}
