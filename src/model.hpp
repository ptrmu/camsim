
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
  };


  struct ModelConfig
  {
    const MarkersConfigurations markers_configuration_;
    const CamerasConfigurations cameras_configuration_;
    const CameraTypes camera_type_;
    double marker_size_;
    double marker_spacing_;
    double camera_spacing_;
    const PoseGenerator marker_pose_generator_;
    const PoseGenerator camera_pose_generator_;

    ModelConfig(MarkersConfigurations markers_configuration,
                CamerasConfigurations cameras_configuration,
                CameraTypes camera_type);

    ModelConfig(PoseGenerator marker_pose_generator,
                PoseGenerator camera_pose_generator,
                CameraTypes camera_type,
                double marker_size);

    ModelConfig(const ModelConfig &model_config) = default;
  };

  class Model;

  class MarkerModel;

  class MarkersModel;

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

    std::size_t marker_index() const;

    std::size_t camera_index() const;
  };


  struct BaseModel
  {
    const ModelConfig cfg_;
//    CamerasModel cameras_;

    explicit BaseModel(const ModelConfig &cfg);
  };

  struct Model : public BaseModel
  {
//    const ModelConfig cfg_;
    const MarkersModel markers_;
    const CamerasModel cameras_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    explicit Model(const ModelConfig &cfg);

    void print_corners_f_image();
  };

  struct BoardConfig
  {
    const int squares_x_;
    const int squares_y_;
    const double square_length_;
    const double marker_length_;

    BoardConfig(int squares_x, int squares_y, double square_length, double marker_length) :
      squares_x_{squares_x}, squares_y_{squares_y},
      square_length_{square_length}, marker_length_{marker_length}
    {}

    BoardConfig(const BoardConfig &n) :
      squares_x_{n.squares_x_}, squares_y_{n.squares_y_},
      square_length_{n.square_length_}, marker_length_{n.marker_length_}
    {}
  };

  struct BoardModel
  {

  };

  struct BoardsModel
  {
    BoardConfig bd_cfg_;
    double square_add_marker_length_;
    double square_sub_marker_length_;

    BoardsModel(const BoardConfig &bd_cfg);
  };

  struct CalibrationModel : public BaseModel
  {
    BoardsModel boards_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    explicit CalibrationModel(const ModelConfig &cfg);

    void print_corners_f_image();
  };
}
#endif //_MODEL_HPP
