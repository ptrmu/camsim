
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
  enum MarkersConfigurations
  {
    square_around_origin_xy_plane = 0,
    single_center,
    single_south_west,
    along_x_axis,
    circle_around_z_axis,
    upright_circle_around_z_axis,
    tetrahedron,
    cube,
    octahedron,
  };

  enum CamerasConfigurations
  {
    z2_facing_origin = 0,
    center_looking_x,
    far_south,
    plus_x_facing_markers,
    square_around_z_axis,
    fly_to_plus_y,
    c_along_x_axis,
  };

  enum CameraTypes
  {
    simple_camera = 0,
    distorted_camera,
  };

  struct ModelConfig
  {
    const MarkersConfigurations markers_configuration_;
    const CamerasConfigurations cameras_configuration_;
    const CameraTypes camera_type_;
    double marker_size_;
    double marker_spacing_;
    double camera_spacing_;

    ModelConfig(MarkersConfigurations markers_configuration,
                CamerasConfigurations cameras_configuration,
                CameraTypes camera_type);

    ModelConfig(const ModelConfig &model_config);
  };

  struct MarkerModel
  {
    const std::size_t marker_idx_;
    const gtsam::Pose3 marker_f_world_;
    const std::vector<gtsam::Point3> corners_f_world_;

    MarkerModel(std::size_t marker_idx,
                const gtsam::Pose3 &marker_f_world,
                std::vector<gtsam::Point3> corners_f_world) :
      marker_idx_{marker_idx},
      marker_f_world_{marker_f_world},
      corners_f_world_{std::move(corners_f_world)}
    {}
  };

  struct MarkersModel
  {
    const ModelConfig &cfg_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    const std::vector<MarkerModel> markers_;

    explicit MarkersModel(const ModelConfig &cfg);

    MarkersModel(const ModelConfig &cfg, const MarkersModel &copy);
  };

  using ProjectFunc = std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                  const gtsam::Point3 &,
                                                  boost::optional<gtsam::Matrix &>)>;

  struct CameraModel
  {
    const std::size_t camera_idx_;
    const gtsam::Pose3 camera_f_world;

    CameraModel(std::size_t camera_idx,
                const gtsam::Pose3 &camera_f_world) :
      camera_idx_{camera_idx},
      camera_f_world{camera_f_world}
    {}
  };

  struct CamerasModel
  {
    const ModelConfig &cfg_;
    const gtsam::Cal3DS2 calibration_;
    const ProjectFunc project_func_;
    const std::vector<CameraModel> cameras_;

    CamerasModel(const ModelConfig &cfg);

    CamerasModel(const ModelConfig &cfg, const gtsam::Pose3 &t_world_base);

    gtsam::Cal3_S2 get_Cal3_S2();
  };

  struct CornersFImageModel
  {
    const std::size_t marker_idx_;
    const std::size_t camera_idx_;
    const std::vector<gtsam::Point2> corners_f_image_;

    CornersFImageModel(std::size_t marker_idx,
                       std::size_t camera_idx,
                       std::vector<gtsam::Point2> corners_f_image) :
      marker_idx_{marker_idx},
      camera_idx_{camera_idx},
      corners_f_image_{std::move(corners_f_image)}
    {}
  };

  struct Model
  {
    ModelConfig cfg_;
    MarkersModel markers_;
    CamerasModel cameras_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    Model(MarkersConfigurations markers_configuration,
          CamerasConfigurations cameras_configuration,
          CameraTypes camera_type);

    Model(const Model &model, const gtsam::Pose3 &t_world_base);

    void print_corners_f_image();
  };
}
#endif //_MODEL_HPP
