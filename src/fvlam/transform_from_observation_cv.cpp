
#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  class TransformFromObservationSolvePnp : public TransformFromObservationInterface
  {
    double marker_length_;

  public:
    TransformFromObservationSolvePnp() = delete;

    explicit TransformFromObservationSolvePnp(double marker_length) :
      marker_length_{marker_length}
    {}

    // calculate the pose of a marker in the camera frame.
    virtual Transform3WithCovariance calc_t_camera_marker(const MarkerObservation &marker_observation,
                                                          const CameraInfo &camera_info)
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame.
      auto corners_f_marker{Marker::to_corners_f_marker<std::vector<cv::Point3d>>(marker_length_)};
      auto corners_f_image{marker_observation.to<std::vector<cv::Point2d>>()};

      // Figure out marker pose.
      cv::Vec3d rvec, tvec;
//      cv::solvePnP(corners_f_marker, corners_f_image,
//                   camera_info.camera_matrix(), camera_info.dist_coeffs(),
//                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return Transform3WithCovariance(Transform3(marker_observation.id(),
                                                 Rotate3::from(rvec),
                                                 Translate3::from(tvec)));
    }

    // Calculate the pose of marker1 in the frame of marker0.
    virtual Transform3WithCovariance calc_t_marker0_marker1(const MarkerObservation &marker_observation,
                                                            const CameraInfo &camera_info)
    {
      return Transform3WithCovariance{};
    }
  };

  std::unique_ptr<TransformFromObservationInterface>
  make_transform_from_observation_solve_pnp(double marker_length)
  {
    return std::make_unique<TransformFromObservationSolvePnp>(marker_length);
  }

}
