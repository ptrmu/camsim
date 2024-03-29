

#include "fvlam/model.hpp"
#include <gtsam/geometry/Cal3DS2.h>
#include "gtsam/inference/Symbol.h"
#include <gtsam/linear/Sampler.h>
#include "opencv2/core.hpp"

namespace fvlam
{
  const double degree = (M_PI / 180.0);

  MapEnvironment MapEnvironmentGen::Default()
  {
    return fvlam::MapEnvironment{"TestMap", 0, 0.2};
  }

  CameraInfoMap CameraInfoMapGen::Simulation()
  {
    auto camera_info_base = CameraInfo{475, 475, 0, 400, 300};
    auto camera_info0 = CameraInfo("imager", camera_info_base, Transform3{});
    auto camera_info_map = CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info0.imager_frame_id(), camera_info0);
    return camera_info_map;
  }

  static CameraInfoMap centered_dual_camera(CameraInfo camera_info_base, double imager_offset)
  {
    auto camera_info0 = CameraInfo("left", camera_info_base,
                                   Transform3{Rotate3{}, Translate3{-imager_offset, 0, 0}});
    auto camera_info1 = CameraInfo("right", camera_info_base,
                                   Transform3{Rotate3{}, Translate3{imager_offset, 0, 0}});

    auto camera_info_map = CameraInfoMap{};
    camera_info_map.m_mutable().emplace(camera_info0.imager_frame_id(), camera_info0);
    camera_info_map.m_mutable().emplace(camera_info1.imager_frame_id(), camera_info1);

    return camera_info_map;
  }

  CameraInfoMap CameraInfoMapGen::Dual()
  {
    auto camera_info_base = CameraInfo{475, 475, 0, 400, 300};
    return centered_dual_camera(camera_info_base, 0.2);
  }

  CameraInfoMap CameraInfoMapGen::DualWideAngle()
  {
    auto camera_info_base = CameraInfo{100, 100, 0, 400, 300};
    return centered_dual_camera(camera_info_base, 0.2);
  }

// ==============================================================================
// CamerasGen class
// ==============================================================================

  static std::vector<fvlam::Transform3> rotate_around_z(int n, const fvlam::Transform3 &base)
  {
    std::vector<fvlam::Transform3> pose_f_worlds{};
    double delta_rotz = M_PI * 2 / n;
    for (int i = 0; i < n; i += 1) {
      auto rotz = delta_rotz * i;
      pose_f_worlds.emplace_back(
        fvlam::Transform3(fvlam::Rotate3::RzRyRx(0., 0., rotz), fvlam::Translate3{0, 0, 0}) * base);
    }
    return pose_f_worlds;
  }

  static std::vector<fvlam::Transform3> circle_in_xy_plane_facing_along_z(int n, double radius, double z_offset,
                                                                          bool facing_z_plus_not_z_negative)
  {
    auto base_rotate = facing_z_plus_not_z_negative ? Rotate3{} : Rotate3::RzRyRx(M_PI, 0, 0);
    auto poses_in_circle = rotate_around_z(n, Transform3{base_rotate, Translate3{radius, 0., z_offset}});
    return poses_in_circle;
  }

  static std::vector<Marker> markers_from_transform3s(std::vector<Transform3> transform3s,
                                                      std::uint64_t id_base)
  {
    std::vector<Marker> markers;
    for (auto &transform3 : transform3s) {
      markers.emplace_back(Marker{id_base++, Transform3WithCovariance{transform3}});
    }
    return markers;
  }

  std::vector<Transform3> CamerasGen::SpinAboutZAtOriginFacingOut(int n)
  {
    Transform3 base{fvlam::Rotate3::RzRyRx(-M_PI_2, 0., -M_PI_2), fvlam::Translate3{0, 0, 0}};
    return rotate_around_z(n, base);
  }

  std::vector<Transform3> CamerasGen::LookingDownZ(double z)
  {
    auto p = Transform3{fvlam::Rotate3::RzRyRx(M_PI, 0., 0.), fvlam::Translate3{0, 0, z}};
    return std::vector<Transform3>{p};
  }

  std::vector<Transform3> CamerasGen::CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                                  bool facing_z_plus_not_z_negative)
  {
    return circle_in_xy_plane_facing_along_z(n, radius, z_offset, facing_z_plus_not_z_negative);
  }

// ==============================================================================
// MarkersGen class
// ==============================================================================

  template<>
  std::vector<Marker> MarkersGen::TargetsFromTransform3s(std::vector<Transform3> transform3s)
  {
    return markers_from_transform3s(transform3s, 0);
  }

  template<>
  std::vector<Marker> MarkersGen::CircleInXYPlaneFacingOrigin(int n, double radius)
  {
    return markers_from_transform3s(rotate_around_z(
      n, Transform3{Rotate3::RzRyRx(M_PI_2, 0., -M_PI_2), Translate3{radius, 0., 0.}}), 0);
  }

  template<>
  std::vector<Marker> MarkersGen::OriginLookingUp(double x)
  {
    return markers_from_transform3s(
      std::vector<Transform3>{
        Transform3{Rotate3{}, Translate3{x, 0, 0}}},
      0);
  }

  template<>
  std::vector<Marker> MarkersGen::OriginLookingUpPlusOne(double x0, double x1)
  {
    return markers_from_transform3s(
      std::vector<Transform3>{
        Transform3{Rotate3{}, Translate3{x0, 0, 0}},
        Transform3{Rotate3{}, Translate3{x1, 0, 0}}},
      0);
  }

  template<>
  std::vector<Marker> MarkersGen::CircleInXYPlaneFacingAlongZ(int n, double radius, double z_offset,
                                                              bool facing_z_plus_not_z_negative)
  {
    return markers_from_transform3s(circle_in_xy_plane_facing_along_z(
      n, radius, z_offset, facing_z_plus_not_z_negative), 0);
  }

// ==============================================================================
// MarkerModelGen class
// ==============================================================================

  // Assuming world coordinate system is ENU
  // Camera coordinate system is Left,down,forward (along camera axis)
  static auto master_marker_pose_list = std::vector<fvlam::Transform3>{
    fvlam::Transform3{0, 0, 0, 0, 0, 0},
    fvlam::Transform3{0, 0, 0, 1, 0, 0},
    fvlam::Transform3{5 * degree, 5 * degree, 0, 1, 1, 0},
    fvlam::Transform3{0, 0, 5 * degree, 0, 1, 0},

    fvlam::Transform3{5 * degree, 0, 0, 0, 0, 0.25},
    fvlam::Transform3{-5 * degree, 0, 0, 1, 1, 0},
  };

  static auto master_camera_pose_list = std::vector<fvlam::Transform3>{
    fvlam::Transform3{180 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{180 * degree, 0, 0, 1, 0, 2},
    fvlam::Transform3{180 * degree, 0, 0, 1, 1, 2},
    fvlam::Transform3{180 * degree, 0, 0, 0, 1, 2},
    fvlam::Transform3{180 * degree, 0, 0, 0.01, 0, 2},

    fvlam::Transform3{181 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{181 * degree, 0, 0, 1, 1, 2},
    fvlam::Transform3{179 * degree, 0, 0, 0, 0, 2},
    fvlam::Transform3{179 * degree, 0, 0, 1, 1, 2},

    fvlam::Transform3{181 * degree, 1 * degree, 1 * degree, 0, 0, 2},
    fvlam::Transform3{181 * degree, 1 * degree, 1 * degree, 1, 1, 2},
    fvlam::Transform3{179 * degree, -1 * degree, -1 * degree, 0, 0, 2},
    fvlam::Transform3{179 * degree, -1 * degree, -1 * degree, 1, 1, 2},
  };

  MarkerModel::Maker MarkerModelGen::MonoSpinCameraAtOrigin()
  {
    return []() -> fvlam::MarkerModel
    {
      int n_cameras = 32;
      int n_markers = 32;
      double z = 2.0;
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Simulation(),
                                fvlam::CamerasGen::SpinAboutZAtOriginFacingOut(n_cameras),
                                fvlam::MarkersGen::CircleInXYPlaneFacingOrigin(n_markers, z));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualSpinCameraAtOrigin()
  {
    return []() -> fvlam::MarkerModel
    {
      int n_cameras = 32;
      int n_markers = 32;
      double z = 2.0;
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Dual(),
                                fvlam::CamerasGen::SpinAboutZAtOriginFacingOut(n_cameras),
                                fvlam::MarkersGen::CircleInXYPlaneFacingOrigin(n_markers, z));
    };
  }

  MarkerModel::Maker MarkerModelGen::MonoParallelGrid()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Simulation(),
                                master_camera_pose_list,
                                fvlam::MarkersGen::TargetsFromTransform3s(master_marker_pose_list));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualParallelGrid()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Dual(),
                                master_camera_pose_list,
                                fvlam::MarkersGen::TargetsFromTransform3s(master_marker_pose_list));
    };
  }

  MarkerModel::Maker MarkerModelGen::MonoParallelCircles()
  {
    return []() -> fvlam::MarkerModel
    {
      int n_cameras = 256;
      int n_markers = 8;
      double z = 2.0;
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Simulation(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  n_cameras, 1.0, z, false),
                                fvlam::MarkersGen::CircleInXYPlaneFacingAlongZ(
                                  n_markers, 1.0, 0.0, true));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualParallelCircles()
  {
    return []() -> fvlam::MarkerModel
    {
      int n_cameras = 32;
      int n_markers = 32;
      double z = 2.0;
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Dual(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  n_cameras, 1.0, z, false),
                                fvlam::MarkersGen::CircleInXYPlaneFacingAlongZ(
                                  n_markers, 1.0, 0.0, true));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualWideSingleCamera()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::DualWideAngle(),
                                fvlam::CamerasGen::LookingDownZ(2.0),
                                fvlam::MarkersGen::CircleInXYPlaneFacingAlongZ(
                                  8, 1.0, 0.0, true));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualWideSingleMarker()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::DualWideAngle(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  8, 1.0, 2.0, false),
                                fvlam::MarkersGen::OriginLookingUp(0.));
    };
  }

  MarkerModel::Maker MarkerModelGen::MonoSingleMarker()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Simulation(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  1, 0.0, 2.0, false),
                                fvlam::MarkersGen::OriginLookingUp(0.0));
    };
  }

  MarkerModel::Maker MarkerModelGen::DualSingleMarker()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Dual(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  1, 0.0, 2.0, false),
                                fvlam::MarkersGen::OriginLookingUp(0.0));
    };
  }

  MarkerModel::Maker MarkerModelGen::MonoDoubleMarker()
  {
    return []() -> fvlam::MarkerModel
    {
      return fvlam::MarkerModel(fvlam::MapEnvironmentGen::Default(),
                                fvlam::CameraInfoMapGen::Simulation(),
                                fvlam::CamerasGen::CircleInXYPlaneFacingAlongZ(
                                  1, 0.0, 2.0, false),
                                fvlam::MarkersGen::OriginLookingUpPlusOne(0., 1.));
    };
  }

  std::uint64_t ModelKey::value(std::size_t value_idx)
  {
    return gtsam::Symbol{key_char_value, value_idx}.key();
  }

  std::uint64_t ModelKey::camera(std::size_t camera_idx)
  {
    return gtsam::Symbol{key_char_camera, camera_idx}.key();
  }

  std::uint64_t ModelKey::marker(std::size_t marker_idx)
  {
    return gtsam::Symbol{key_char_marker, marker_idx}.key();
  }

  static const std::size_t camera_marker_bits = 24;
  static const std::size_t camera_marker_stride = 1 << camera_marker_bits;
  static const std::uint64_t camera_marker_mask = camera_marker_stride - 1;

  std::uint64_t ModelKey::camera_marker(std::size_t camera_idx, std::size_t marker_idx)
  {
    return gtsam::Symbol{key_char_camera_marker, camera_idx * camera_marker_stride + marker_idx}.key();
  }

  std::uint64_t ModelKey::marker_marker(std::size_t marker0_idx, std::size_t marker1_idx)
  {
    return gtsam::Symbol{key_char_marker_marker, marker0_idx * camera_marker_stride + marker1_idx}.key();
  }

  std::uint64_t ModelKey::corner(std::uint64_t marker_key, std::size_t corner_idx)
  {
    static char codes[] = {key_char_corner0, key_char_corner1, key_char_corner2, key_char_corner3};
    auto marker_index = gtsam::Symbol{marker_key}.index();
    return gtsam::Symbol(codes[corner_idx % sizeof(codes)], marker_index);
  }

  std::uint64_t ModelKey::marker_from_corner(std::uint64_t corner_key)
  {
    return marker(gtsam::Symbol{corner_key}.index());
  }

  std::size_t ModelKey::camera_idx_from_camera_marker(std::uint64_t camera_marker_key)
  {
    assert(gtsam::Symbol{camera_marker_key}.chr() == key_char_camera_marker);
    return (gtsam::Symbol{camera_marker_key}.index() >> camera_marker_bits) & camera_marker_mask;
  }

  std::size_t ModelKey::marker_idx_from_camera_marker(std::uint64_t camera_marker_key)
  {
    assert(gtsam::Symbol{camera_marker_key}.chr() == key_char_camera_marker);
    return gtsam::Symbol{camera_marker_key}.index() & camera_marker_mask;
  }

  std::size_t ModelKey::id0_from_marker_marker(std::uint64_t marker_marker_key)
  {
    assert(gtsam::Symbol{marker_marker_key}.chr() == key_char_marker_marker);
    return (gtsam::Symbol{marker_marker_key}.index() >> camera_marker_bits) & camera_marker_mask;
  }

  std::size_t ModelKey::id1_from_marker_marker(std::uint64_t marker_marker_key)
  {
    assert(gtsam::Symbol{marker_marker_key}.chr() == key_char_marker_marker);
    return gtsam::Symbol{marker_marker_key}.index() & camera_marker_mask;
  }

// ==============================================================================
// MarkerObservations class
// ==============================================================================

  ObservationsSynced MarkerObservations::gen_observations_synced(const MapEnvironment &map_environment,
                                                                 const CameraInfoMap &camera_info_map,
                                                                 const Transform3 &t_map_camera,
                                                                 const std::vector<Marker> &markers)
  {
    auto observations_synced = ObservationsSynced{Stamp{}, "camera_frame"};

    for (auto &camera_info_pair : camera_info_map.m()) {
      const CameraInfo &camera_info = camera_info_pair.second;
      auto gtsam_camera_calibration = camera_info.to<gtsam::Cal3DS2>();

      auto project_t_world_marker_function = fvlam::Marker::project_t_world_marker(
        gtsam_camera_calibration,
        t_map_camera * camera_info.t_camera_imager(),
        map_environment.marker_length());

      auto observations = Observations{camera_info.imager_frame_id()};
      for (auto &marker : markers) {
        auto observation = project_t_world_marker_function(marker);
        if (observation.is_valid()) {
          observations.v_mutable().emplace_back(observation);
        }
      }
      observations_synced.v_mutable().emplace_back(observations);
    }

    return observations_synced;
  }

// ==============================================================================
// MarkerModelRunner class
// ==============================================================================

  MarkerModelRunner::MarkerModelRunner(Config cfg,
                                       MarkerModel::Maker model_maker) :
    cfg_{cfg}, logger_{cfg.logger_level_}, model_{model_maker()},
    map_{gen_map(model_)},
    markers_perturbed_{gen_markers_perturbed(model_, cfg.r_sampler_sigma_, cfg.t_sampler_sigma_)},
    marker_observations_list_perturbed_{
      gen_marker_observations_list_perturbed(model_, cfg.r_sampler_sigma_, cfg.t_sampler_sigma_, cfg.u_sampler_sigma_)}
  {
    logger_.info() << "Model Markers Perturbed:";
    for (auto &marker : markers_perturbed_) {
      logger_.info() << marker.to_string();
    }

    logger_.debug() << "Model Observations Perturbed:";
    for (auto &to : marker_observations_list_perturbed_) {
      logger_.debug() << "ObservationsSynced " << to.camera_index() << " "
                      << to.t_map_camera().to_string();
      for (auto &os : to.observations_synced().v()) {
        for (auto &o : os.v()) {
          auto &cs = o.corners_f_image();
          logger_.info() << os.imager_frame_id() << " "
                         << o.id() << " ("
                         << cs[0].t().transpose() << ") ("
                         << cs[1].t().transpose() << ") ( "
                         << cs[2].t().transpose() << ") ("
                         << cs[3].t().transpose() << ")";
        }
      }
    }
  }

  fvlam::MarkerMap MarkerModelRunner::gen_map(
    const fvlam::MarkerModel &model)
  {
    auto map = fvlam::MarkerMap{model.environment()};
    for (auto &marker : model.targets()) {
      map.add_marker(marker);
    }
    return map;
  }

  std::vector<Marker> MarkerModelRunner::gen_markers_perturbed(
    const fvlam::MarkerModel &model,
    double r_sampler_sigma,
    double t_sampler_sigma)
  {
    auto pose3_sampler = gtsam::Sampler{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(r_sampler_sigma),
        fvlam::Translate3::MuVector::Constant(t_sampler_sigma)).finished()};

    auto markers_perturbed = std::vector<Marker>{};
    for (auto &target_marker : model.targets()) {
      auto tf_perturbed = Transform3WithCovariance{target_marker.t_map_marker().tf().retract(pose3_sampler.sample()),
                                                   target_marker.t_map_marker().cov()};
      markers_perturbed.emplace_back(Marker{target_marker.id(), tf_perturbed, target_marker.is_valid()});
    }
    return markers_perturbed;
  }

  std::vector<fvlam::MarkerObservations> MarkerModelRunner::gen_marker_observations_list_perturbed(
    const fvlam::MarkerModel &model,
    double r_sampler_sigma,
    double t_sampler_sigma,
    double point2_sampler_sigma)
  {
    auto point2_sampler = gtsam::Sampler{fvlam::Translate2::MuVector::Constant(point2_sampler_sigma)};
    auto pose3_sampler = gtsam::Sampler{
      (fvlam::Transform3::MuVector{} << fvlam::Rotate3::MuVector::Constant(r_sampler_sigma),
        fvlam::Translate3::MuVector::Constant(t_sampler_sigma)).finished()};

    std::vector<fvlam::MarkerObservations> marker_observations_list_perturbed{};

    for (auto const &target_observations: model.target_observations_list()) {
      auto &observations_synced = target_observations.observations_synced();

      fvlam::ObservationsSynced perturbed_observations_synced{observations_synced.stamp(),
                                                              observations_synced.camera_frame_id()};
      for (auto &observations : observations_synced.v()) {

        fvlam::Observations perturbed_observations{observations.imager_frame_id()};
        for (auto &observation : observations.v()) {

          auto cfi = observation.corners_f_image();
          auto corners_f_image_perturbed = fvlam::Observation::Array{
            fvlam::Translate2{cfi[0].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[1].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[2].t() + point2_sampler.sample()},
            fvlam::Translate2{cfi[3].t() + point2_sampler.sample()},
          };
          fvlam::Observation perturbed_observation{observation.id(),
                                                   corners_f_image_perturbed,
                                                   observation.cov()};
          perturbed_observations.v_mutable().emplace_back(perturbed_observation);
        }
        perturbed_observations_synced.v_mutable().emplace_back(perturbed_observations);
      }
      fvlam::MarkerObservations perturbed_marker_observations{
        target_observations.camera_index(),
        target_observations.t_map_camera().retract(pose3_sampler.sample()),
        perturbed_observations_synced};
      marker_observations_list_perturbed.emplace_back(perturbed_marker_observations);
    }

    return marker_observations_list_perturbed;
  }


  int MarkerModelRunner::for_each_marker_observations(bool truth_not_perturbed,
                                                      std::function<int(const fvlam::MarkerObservations &)> cb)
  {
    // For each camera location
    for (auto &marker_observations : truth_not_perturbed ?
                                     model_.target_observations_list() :
                                     marker_observations_list_perturbed_) {

      auto ret = cb(marker_observations);

      if (ret != 0) {
        return ret;
      }
    }
    return 0;
  }

  int MarkerModelRunner::for_each_observations(bool truth_not_perturbed,
                                               std::function<int(const fvlam::MarkerObservations &,
                                                                 const fvlam::Observations &,
                                                                 const fvlam::CameraInfo &)> cb)
  {
    return for_each_marker_observations(truth_not_perturbed,
                                        [this, &cb](const fvlam::MarkerObservations &marker_observations) -> int
                                        {
                                          // For each imager's observations
                                          for (auto &observations : marker_observations.observations_synced().v()) {

                                            // Find the camera_info
                                            const auto &kp = model_.camera_info_map().m().find(
                                              observations.imager_frame_id());
                                            if (kp == model_.camera_info_map().m().end()) {
                                              continue;
                                            }

                                            auto ret = cb(marker_observations,
                                                          observations,
                                                          kp->second);

                                            if (ret != 0) {
                                              return ret;
                                            }
                                          }
                                          return 0;
                                        });
  }

  int MarkerModelRunner::for_each_observation(bool truth_not_perturbed,
                                              std::function<int(const fvlam::MarkerObservations &,
                                                                const fvlam::Observations &,
                                                                const fvlam::CameraInfo &,
                                                                const fvlam::Observation &)> cb)
  {
    return for_each_observations(truth_not_perturbed,
                                 [&cb](const fvlam::MarkerObservations &marker_observations,
                                       const fvlam::Observations &observations,
                                       const fvlam::CameraInfo &camera_info) -> int
                                 {
                                   // For each observation of a marker
                                   for (auto &observation : observations.v()) {

                                     auto ret = cb(marker_observations,
                                                   observations,
                                                   camera_info,
                                                   observation);

                                     if (ret != 0) {
                                       return ret;
                                     }
                                   }
                                   return 0;
                                 });
  }

  int MarkerModelRunner::for_each_corner_f_image(bool truth_not_perturbed,
                                                 std::function<int(const fvlam::MarkerObservations &,
                                                                   const fvlam::Observations &,
                                                                   const fvlam::CameraInfo &,
                                                                   const fvlam::Observation &,
                                                                   std::size_t,
                                                                   const fvlam::Translate2)> cb)
  {
    return for_each_observation(truth_not_perturbed,
                                [&cb](const fvlam::MarkerObservations &marker_observations,
                                      const fvlam::Observations &observations,
                                      const fvlam::CameraInfo &camera_info,
                                      const fvlam::Observation &observation) -> int
                                {
                                  auto corners_f_image = observation.corners_f_image();

                                  // For each corner measurement
                                  for (std::size_t i = 0; i < Marker::ArraySize; i += 1) {

                                    auto ret = cb(marker_observations,
                                                  observations,
                                                  camera_info,
                                                  observation,
                                                  i,
                                                  corners_f_image[i]);

                                    if (ret != 0) {
                                      return ret;
                                    }
                                  }
                                  return 0;
                                });
  }


  int MarkerModelRunner::for_all_marker_observations(bool truth_not_perturbed,
                                                     const std::function<int(const MarkerObservations &)> &cb)
  {
    for (auto famos = make_for_all_marker_observations(truth_not_perturbed); famos.test(); famos.next()) {
      auto ret = cb(famos.marker_observations());
      if (ret != 0) {
        return ret;
      }
    }
    return 0;
  }

  int MarkerModelRunner::for_all_observations(bool truth_not_perturbed,
                                              const std::function<int(const fvlam::MarkerObservations &,
                                                                      const fvlam::Observations &,
                                                                      const fvlam::CameraInfo &)> &cb)
  {
    for (auto famos = make_for_all_marker_observations(truth_not_perturbed); famos.test(); famos.next()) {
      for (auto faos = ForAllObservations(famos.marker_observations(), model_.camera_info_map());
           faos.test(); faos.next()) {
        auto ret = cb(famos.marker_observations(),
                      faos.observations(),
                      faos.camera_info());
        if (ret != 0) {
          return ret;
        }
      }
    }
    return 0;
  }

  int MarkerModelRunner::for_all_observation(bool truth_not_perturbed,
                                             const std::function<int(const fvlam::MarkerObservations &,
                                                                     const fvlam::Observations &,
                                                                     const fvlam::CameraInfo &,
                                                                     const fvlam::Observation &)> &cb)
  {
    for (auto famos = make_for_all_marker_observations(truth_not_perturbed); famos.test(); famos.next()) {
      for (auto faos = ForAllObservations(famos.marker_observations(), model_.camera_info_map());
           faos.test(); faos.next()) {
        for (auto fao = ForAllObservation(faos.observations()); fao.test(); fao.next()) {
          auto ret = cb(famos.marker_observations(),
                        faos.observations(),
                        faos.camera_info(),
                        fao.observation());
          if (ret != 0) {
            return ret;
          }
        }
      }
    }
    return 0;
  }

  int MarkerModelRunner::for_all_corner_f_image(bool truth_not_perturbed,
                                                const std::function<int(const fvlam::MarkerObservations &,
                                                                        const fvlam::Observations &,
                                                                        const fvlam::CameraInfo &,
                                                                        const fvlam::Observation &,
                                                                        std::size_t,
                                                                        const fvlam::Translate2)> &cb)
  {
    for (auto famos = make_for_all_marker_observations(truth_not_perturbed); famos.test(); famos.next()) {
      for (auto faos = ForAllObservations(famos.marker_observations(), model_.camera_info_map());
           faos.test(); faos.next()) {
        for (auto fao = ForAllObservation(faos.observations()); fao.test(); fao.next()) {
          for (auto facfi = ForAllCornerFImage(fao.observation().corners_f_image()); facfi.test(); facfi.next()) {
            auto ret = cb(famos.marker_observations(),
                          faos.observations(),
                          faos.camera_info(),
                          fao.observation(),
                          facfi.corner_index(),
                          facfi.corner_f_image());
            if (ret != 0) {
              return ret;
            }
          }
        }
      }
    }
    return 0;
  }
}
