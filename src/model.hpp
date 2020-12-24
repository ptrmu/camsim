
#ifndef _MODEL_HPP
#define _MODEL_HPP

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  // The world coordinate frame is ENU.
  // The Marker Coordinate frame is centered on the marker, x to the right, y to the top, z out of the marker
  // The camera coordinate frame has z pointing in the direction it looks, x to the right and y down
  // The image coordinate frame has two dimensions, u,v. u aligns with camera x, v aligns with camera y
  //  the center of the image is aligned with the origin of the camera.

  using PoseGenerator = std::function<std::vector<gtsam::Pose3>()>;

  struct PoseGens
  {
    class Noop
    {
    public:
      std::vector<gtsam::Pose3> operator()() const;
    };

    class CircleInXYPlaneFacingOrigin
    {
      int n_;
      double radius_;
    public:
      CircleInXYPlaneFacingOrigin(int n, double radius) :
        n_{n}, radius_{radius}
      {}

      std::vector<gtsam::Pose3> operator()() const;
    };

    class SpinAboutZAtOriginFacingOut
    {
      int n_;
    public:
      explicit SpinAboutZAtOriginFacingOut(int n) :
        n_{n}
      {}

      std::vector<gtsam::Pose3> operator()() const;
    };

    class CircleInXYPlaneFacingAlongZ
    {
      int n_;
      double radius_;
      double z_offset_;
      bool facing_z_plus_not_z_negative_;

    public:
      explicit CircleInXYPlaneFacingAlongZ(int n,
                                           double radius,
                                           double z_offset,
                                           bool facing_z_plus_not_z_negative) :
        n_{n}, radius_{radius}, z_offset_{z_offset}, facing_z_plus_not_z_negative_{facing_z_plus_not_z_negative}
      {}

      std::vector<gtsam::Pose3> operator()() const;
    };

    class CubeAlongZFacingOrigin
    {
      int per_side_;
      double side_length_;
      double z_offset_;

    public:
      explicit CubeAlongZFacingOrigin(int per_side,
                                      double side_length,
                                      double z_offset) :
        per_side_{per_side}, side_length_{side_length}, z_offset_{z_offset}
      {}

      std::vector<gtsam::Pose3> operator()() const;
    };


    using PoseGeneratorFunc = std::function<std::vector<gtsam::Pose3>(void)>;

    static PoseGeneratorFunc gen_poses_func(std::vector<gtsam::Pose3> poses)
    {
      return [poses]()
      {
        return poses;
      };
    }

    static PoseGeneratorFunc gen_poses_func_origin_looking_up()
    {
      return gen_poses_func({gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., 0.),
                                          gtsam::Point3{0., 0., 0.}}});
    }

    static PoseGeneratorFunc gen_poses_func_heiko_calibration_poses()
    {
      return gen_poses_func({gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                          gtsam::Point3{0., 0., 0.18}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(0.75 * M_PI, 0., M_PI),
                                          gtsam::Point3{0., -0.18, 0.18}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(-0.75 * M_PI, 0., M_PI),
                                          gtsam::Point3{0., 0.18, 0.18}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0.25 * M_PI, M_PI),
                                          gtsam::Point3{-0.195, 0., 0.195}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, -0.25 * M_PI, M_PI),
                                          gtsam::Point3{0.195, 0., 0.195}}});
    }

    static PoseGeneratorFunc gen_poses_func_homography_calibration_poses()
    {
      return gen_poses_func({gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                          gtsam::Point3{0., 0., 1.}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(0.75 * M_PI, 0., M_PI),
                                          gtsam::Point3{0., -0., 1.}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(-0.75 * M_PI, 0., M_PI),
                                          gtsam::Point3{0., 0., 1.}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0.25 * M_PI, M_PI),
                                          gtsam::Point3{-0., 0., 1.}},
                             gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, -0.25 * M_PI, M_PI),
                                          gtsam::Point3{0., 0., 1.}}});
    }
  };

  enum class MarkersConfigurations
  {
    generator = 0,
    square_around_origin_xy_plane,
    single_center,
    single_south_west,
    along_x_axis,
    circle_around_z_axis,
    upright_circle_around_z_axis,
    tetrahedron,
    cube,
    octahedron,
  };

  enum class CamerasConfigurations
  {
    generator = 0,
    z2_facing_origin,
    center_looking_x,
    far_south,
    plus_x_facing_markers,
    square_around_z_axis,
    fly_to_plus_y,
    c_along_x_axis,
  };

  enum class CameraTypes
  {
    simple_camera = 0,
    distorted_camera,
    simulation,
    custom,
  };


  struct ModelConfig
  {
    const MarkersConfigurations markers_configuration_;
    const CamerasConfigurations cameras_configuration_;
    const CameraTypes camera_type_;
    const gtsam::Cal3DS2 cal3ds2_;
    const double marker_length_;
    const double marker_spacing_;
    const double camera_spacing_;
    const PoseGenerator marker_pose_generator_;
    const PoseGenerator camera_pose_generator_;
    const bool do_not_rotate_cameras_;

    ModelConfig(MarkersConfigurations markers_configuration,
                CamerasConfigurations cameras_configuration,
                CameraTypes camera_type);

    ModelConfig(PoseGenerator marker_pose_generator,
                PoseGenerator camera_pose_generator,
                CameraTypes camera_type,
                double marker_length,
                bool do_not_rotate_cameras = false);

    ModelConfig(PoseGenerator marker_pose_generator,
                PoseGenerator camera_pose_generator,
                const gtsam::Cal3DS2 &cal3ds2);

    ModelConfig(const ModelConfig &model_config) = default;
  };

  struct CornerModel
  {
    const std::uint64_t key_;
    const gtsam::Point3 point_f_world_;

    CornerModel(std::uint64_t key,
                gtsam::Point3 point_f_world) :
      key_{key},
      point_f_world_{std::move(point_f_world)}
    {}

    std::size_t index() const;

    static std::uint64_t default_key(int corner_idx);

    static std::uint64_t corner_key(std::uint64_t marker_key, int corner_idx);
  };

  struct CornersModel
  {
    const std::array<CornerModel, 4> corners_;

    CornersModel(std::uint64_t marker_key_,
                 const gtsam::Pose3 &marker_f_world,
                 const std::vector<gtsam::Point3> &corners_f_marker);
  };

  struct MarkerModel
  {
    const std::uint64_t key_;
    const gtsam::Pose3 marker_f_world_;
    const CornersModel corners_;
    const std::vector<gtsam::Point3> corners_f_world_; // remove this at sometime

    MarkerModel(const std::uint64_t key,
                const gtsam::Pose3 &marker_f_world,
                CornersModel corners,
                std::vector<gtsam::Point3> corners_f_world) :
      key_{key},
      marker_f_world_{marker_f_world},
      corners_{std::move(corners)},
      corners_f_world_{std::move(corners_f_world)}
    {}

    std::size_t index() const;

    static std::uint64_t default_key();

    static std::uint64_t marker_key(std::size_t idx);

    static std::uint64_t marker_key_from_corner_key(std::uint64_t corner_key);
  };

  struct MarkersModel
  {
    const ModelConfig &cfg_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    const std::vector<MarkerModel> markers_;

    explicit MarkersModel(const ModelConfig &cfg);
  };

  using ProjectFunc = std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                  const gtsam::Point3 &,
                                                  boost::optional<gtsam::Matrix &>)>;

  struct CameraModel
  {
    const std::uint64_t key_;
    const gtsam::Pose3 camera_f_world_;

    CameraModel(const std::uint64_t key,
                const gtsam::Pose3 &camera_f_world) :
      key_{key},
      camera_f_world_{camera_f_world}
    {}

    std::size_t index() const;

    static std::uint64_t default_key();

    static std::uint64_t camera_key(std::size_t idx);
  };

  struct CamerasModel
  {
    const ModelConfig &cfg_;
    const gtsam::Cal3DS2 calibration_;
    const ProjectFunc project_func_;
    const std::vector<CameraModel> cameras_;

    explicit CamerasModel(const ModelConfig &cfg);

    gtsam::Cal3_S2 get_Cal3_S2() const;
  };

  struct CornersFImageModel
  {
    const std::uint64_t marker_key_;
    const std::uint64_t camera_key_;
    const std::vector<gtsam::Point2> corners_f_image_;

    CornersFImageModel(std::uint64_t marker_key,
                       std::uint64_t camera_key,
                       std::vector<gtsam::Point2> corners_f_image) :
      marker_key_{marker_key},
      camera_key_{camera_key},
      corners_f_image_{std::move(corners_f_image)}
    {}

    std::size_t marker_index() const; //
    std::size_t camera_index() const; //
  };


  struct BaseModel
  {
    const ModelConfig cfg_;
    const CamerasModel cameras_;

    explicit BaseModel(const ModelConfig &cfg);
  };

  struct Model : public BaseModel
  {
    const MarkersModel markers_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    explicit Model(const ModelConfig &cfg);

    void print_corners_f_image();
  };
}
#endif //_MODEL_HPP
