
#ifndef _MODEL_HPP
#define _MODEL_HPP

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  // The world coordinate frame is ENU.
  // The Marker Coordinate frame is centered on the marker, x to the right, y to the top, x out of the marker
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
  };

  struct MarkerModel
  {
    const std::size_t marker_idx_;
    const gtsam::Pose3 pose_f_world_;
    const std::vector<gtsam::Point3> corners_f_world_;

    MarkerModel(std::size_t marker_idx,
                const gtsam::Pose3 &pose_f_world,
                std::vector<gtsam::Point3> corners_f_world) :
      marker_idx_{marker_idx},
      pose_f_world_{pose_f_world},
      corners_f_world_{std::move(corners_f_world)}
    {}
  };

  struct MarkersModel
  {
    const MarkersConfigurations markers_configuration_;
    const double marker_size_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    const std::vector<MarkerModel> markers_;

    explicit MarkersModel(MarkersConfigurations markers_configuration);
  };

  enum CamerasConfigurations
  {
    center_facing_markers = 0,
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

  struct CameraModel
  {
    const std::size_t camera_idx_;
    const gtsam::Pose3 pose_f_world_;
    const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                      const gtsam::Point3 &,
                                      boost::optional<gtsam::Matrix &>)> project_func_;

    CameraModel(std::size_t camera_idx,
                const gtsam::Pose3 &pose_f_world,
                const std::function<gtsam::Point2(const gtsam::Pose3 &,
                                                  const gtsam::Point3 &,
                                                  boost::optional<gtsam::Matrix &>)> &project_func) :
      camera_idx_{camera_idx},
      pose_f_world_{pose_f_world},
      project_func_{project_func}
    {}
  };

  struct CamerasModel
  {
    const CameraTypes camera_type_;
    const CamerasConfigurations cameras_configuration_;
    const gtsam::Cal3DS2 calibration_;
    const std::vector<CameraModel> cameras_;

    CamerasModel(CameraTypes camera_type,
                 CamerasConfigurations cameras_configuration,
                 double marker_size);

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
    MarkersModel markers_;
    CamerasModel cameras_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    Model(MarkersConfigurations markers_configuration,
          CamerasConfigurations cameras_configuration,
          CameraTypes camera_type);


  };
}
#endif //_MODEL_HPP
