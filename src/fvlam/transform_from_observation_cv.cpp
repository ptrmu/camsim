
#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  template<>
  std::vector<cv::Point2d> MarkerObservation::to<std::vector<cv::Point2d>>() const
  {
    return std::vector<cv::Point2d>{
      cv::Point2d{corners_f_image_(0, 0), corners_f_image_(1, 0)},
      cv::Point2d{corners_f_image_(0, 1), corners_f_image_(1, 1)},
      cv::Point2d{corners_f_image_(0, 2), corners_f_image_(1, 2)},
      cv::Point2d{corners_f_image_(0, 3), corners_f_image_(1, 3)}
    };
  }

  template<>
  std::vector<cv::Point3d> Marker::to_corners_f_marker<std::vector<cv::Point3d>>(double marker_length)
  {
    auto corners_f_marker = calc_corners_f_marker(marker_length);
    return std::vector<cv::Point3d>{
      cv::Point3d{corners_f_marker(0, 0), corners_f_marker(1, 0), corners_f_marker(2, 0)},
      cv::Point3d{corners_f_marker(0, 1), corners_f_marker(1, 1), corners_f_marker(2, 1)},
      cv::Point3d{corners_f_marker(0, 2), corners_f_marker(1, 2), corners_f_marker(2, 2)},
      cv::Point3d{corners_f_marker(0, 3), corners_f_marker(1, 3), corners_f_marker(2, 3)}
    };
  }

//  template<>
//  Transform3 Transform3::from(const std::pair<cv::Vec3d, cv::Vec3d> &rvec_tvec)
//  {
//    cv::Mat rmat;
//    cv::Rodrigues(rvec_tvec.first, rmat);
//    Rotate3::RotationMatrix m;
//    for (int row = 0; row < 3; row++) {
//      for (int col = 0; col < 3; col++) {
//        m(row, col) = rmat.at<double>(row, col);  // Row- vs. column-major order
//      }
//    }
//    return Transform3(m, tf2::Vector3{tvec[0], tvec[1], tvec[2]});
//  }

  template<>
  Rotate3 Rotate3::from(const cv::Vec3d &other)
  {
    cv::Mat rmat;
    cv::Rodrigues(other, rmat);
    Rotate3::RotationMatrix m;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        m(row, col) = rmat.at<double>(row, col);  // Row- vs. column-major order
      }
    }
    return Rotate3{m};
  }

  template<>
  Translate3 Translate3::from(const cv::Vec3d &other)
  {
    return Translate3{(Translate3::MuVector() << other[0], other[1], other[2]).finished()};
  }

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
