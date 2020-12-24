
#include <memory>
#include <fvlam/camera_info.hpp>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/marker_map.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/sfm/ShonanAveraging.h>
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  template<class MUVECTOR>
  class EstimateMeanAndCovariance
  {
    std::uint64_t id_;
    MUVECTOR first_sample_{MUVECTOR::Zero()};
    MUVECTOR mu_sum_{MUVECTOR::Zero()};
    std::uint64_t n_{0};

  public:
    explicit EstimateMeanAndCovariance(std::uint64_t id) :
      id_{id}
    {}

    void accumulate(const MUVECTOR &mu)
    {
      if (n_ == 0) {
        first_sample_ = mu;
      }
      mu_sum_ += mu - first_sample_;
      n_ += 1;
    }

    MUVECTOR mean() const
    {
      return mu_sum_ / n_ + first_sample_;
    }
  };

  class BuildMarkerMapShonan : public BuildMarkerMapInterface
  {
    const BuildMarkerMapShonanContext context_;
    const fvlam::Marker fixed_marker_;
    const double marker_length_;

    std::map<std::uint64_t, EstimateMeanAndCovariance<Transform3::MuVector>> t_marker0_marker1_map_;


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
      auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(Rotate3::MuVector::Ones() * 0.1);

      for (auto it = t_marker0_marker1_map_.begin(); it != t_marker0_marker1_map_.end(); ++it) {
        std::uint64_t id0 = it->first / 1000000L;
        std::uint64_t id1 = it->first % 1000000L;

        auto mean = Transform3{it->first, it->second.mean()};
        std::cout << "id0:" << id0 << " id1:" << id1
                  << " mean " << mean.to_string(true) << std::endl;
        auto measurement = gtsam::BinaryMeasurement<gtsam::Rot3>{
          id0, id1, mean.r().to<gtsam::Rot3>(), noiseModel};
        measurements.emplace_back(measurement);
      }

      return measurements;
    }

    gtsam::NonlinearFactorGraph load_pose_graph()
    {
      gtsam::NonlinearFactorGraph pose_graph{};
      auto noiseModel = gtsam::noiseModel::Diagonal::Sigmas(Transform3::MuVector::Ones() * 0.1);

      for (auto it = t_marker0_marker1_map_.begin(); it != t_marker0_marker1_map_.end(); ++it) {
        std::uint64_t id0 = it->first / 1000000L;
        std::uint64_t id1 = it->first % 1000000L;

        auto mean = Transform3{it->second.mean()};
        pose_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          id0, id1, mean.to<gtsam::Pose3>(), noiseModel);
      }

      // Add the prior for the fixed node.
      pose_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        fixed_marker_.id(), fixed_marker_.t_world_marker().tf().to<gtsam::Pose3>(), noiseModel);

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
        auto initializedPose = gtsam::Pose3(r_world_shonan * rot, gtsam::Pose3::Translation::Zero());
        pose_initial.insert(key, initializedPose);
      }

      return pose_initial;
    }

    std::unique_ptr<MarkerMap> load_map(const gtsam::Values &pose_result)
    {
      auto map = std::make_unique<MarkerMap>(marker_length_);

      for (const auto &key_value : pose_result) {
        gtsam::Key key = key_value.key;
        const auto &pose = pose_result.at<gtsam::Pose3>(key);
        map->add_marker(Marker{key, Transform3WithCovariance{Transform3::from(pose)}});
      }

      return map;
    }

  public:
    BuildMarkerMapShonan() = delete;

    BuildMarkerMapShonan(BuildMarkerMapShonanContext context,
                         const MarkerMap &map) :
      context_{std::move(context)},
      fixed_marker_{find_fixed_marker(map)},
      marker_length_{map.marker_length()}
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

          // calculate the relative transform between the two markers
          auto t_marker0_marker1 = solve_t_marker0_marker1(obs[m0r], obs[m1r]);

          // Accumulate this in the map
          auto it = t_marker0_marker1_map_.find(t_marker0_marker1.id());
          if (it == t_marker0_marker1_map_.end()) {
            EstimateMeanAndCovariance<Transform3::MuVector> new_mean{t_marker0_marker1.id()};
            auto pair = t_marker0_marker1_map_.emplace(t_marker0_marker1.id(), new_mean);
            assert(pair.second);
            it = pair.first;
          }
          it->second.accumulate(t_marker0_marker1.tf().mu());
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
      shonan_result.first.print("\nshonan_result\n");

      // Prepare for the full Pose optimization
      auto pose_graph = load_pose_graph();
      auto pose_initial = load_pose_initial(shonan_result.first);

      pose_graph.print("pose_graph\n");
      pose_initial.print("pose_initial");

      // Do the pose optimization
      gtsam::GaussNewtonParams params;
      params.setVerbosity("TERMINATION");
      gtsam::GaussNewtonOptimizer optimizer(pose_graph, pose_initial, params);
      auto pose_result = optimizer.optimize();

      return load_map(pose_result);
    }

    std::string reset(std::string &cmd) override
    {
      return std::string{};
    }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapShonanContext>(
    const BuildMarkerMapShonanContext &context,
    const MarkerMap &map_initial)
  {
    return std::make_unique<BuildMarkerMapShonan>(context, map_initial);
  }
}
