
#ifndef _SFM_MODEL_HPP
#define _SFM_MODEL_HPP

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SimpleCamera.h>

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

  struct MarkersModel
  {
    const MarkersConfigurations markers_configuration_;
    const double marker_size_;
    const std::vector<gtsam::Point3> corners_f_marker_;
    const std::vector<gtsam::Pose3> pose_f_worlds_;
    const std::vector<std::vector<gtsam::Point3>> corners_f_worlds_;

    MarkersModel(MarkersConfigurations markers_configuration);
  };

  enum CamerasConfigurations
  {
    center_facing_markers = 0,
    square_around_z_axis,
    fly_to_plus_y,
  };

  struct CamerasModel
  {
    const CamerasConfigurations camera_configuration_;
    const gtsam::Cal3_S2 calibration_;
    const std::vector<gtsam::Pose3> pose_f_worlds_;
    const std::vector<gtsam::SimpleCamera> cameras_;
    const std::vector<std::vector<std::vector<gtsam::Point2>>> corners_f_images_;

    CamerasModel(CamerasConfigurations cameras_configuration,
                 double marker_size,
                 const std::vector<std::vector<gtsam::Point3>> &corners_f_worlds);
  };

  struct SfmModel
  {
    MarkersModel markers_;
    CamerasModel cameras_;

    SfmModel(MarkersConfigurations markers_configuration,
             CamerasConfigurations cameras_configuration) :
      markers_{markers_configuration},
      cameras_{cameras_configuration, markers_.marker_size_, markers_.corners_f_worlds_}
    {}
  };
}
#endif //_SFM_MODEL_HPP
