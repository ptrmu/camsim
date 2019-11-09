
#ifndef _SFM_MODEL_HPP
#define _SFM_MODEL_HPP

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include "sfm_pose_with_covariance.hpp"

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
    east_facing_markers,
    square_around_z_axis,
    fly_to_plus_y,
  };

  struct CameraModel
  {
    const std::size_t camera_idx_;
    const gtsam::Pose3 pose_f_world_;
    const gtsam::SimpleCamera simple_camera_;

    CameraModel(std::size_t camera_idx,
                const gtsam::Pose3 &pose_f_world,
                gtsam::SimpleCamera simple_camera) :
      camera_idx_{camera_idx},
      pose_f_world_{pose_f_world},
      simple_camera_{std::move(simple_camera)}
    {}
  };

  struct CamerasModel
  {
    const CamerasConfigurations cameras_configuration_;
    const gtsam::Cal3_S2 calibration_;
    const std::vector<CameraModel> cameras_;

    CamerasModel(CamerasConfigurations cameras_configuration,
                 double marker_size);
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

  struct SfmModel
  {
    MarkersModel markers_;
    CamerasModel cameras_;
    const std::vector<std::vector<CornersFImageModel>> corners_f_images_;

    SfmModel(MarkersConfigurations markers_configuration,
             CamerasConfigurations cameras_configuration);

    std::string to_str(const SfmPoseWithCovariance &pose_cov);
  };
}
#endif //_SFM_MODEL_HPP
