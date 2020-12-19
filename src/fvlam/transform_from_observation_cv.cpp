
#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{
  template<>
  Translate3 Translate3::from<cv::Vec3d>(const cv::Vec3d &other)
  {
    return Translate3{(Translate3::MuVector() << other[0], other[1], other[2]).finished()};
  }

  template<>
  cv::Vec3d Translate3::to<cv::Vec3d>() const
  {
    return cv::Vec3d{t()(0), t()(1), t()(2)};
  }

  template<>
  Rotate3 Rotate3::from<cv::Vec3d>(const cv::Vec3d &other)
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
  cv::Vec3d Rotate3::to<cv::Vec3d>() const
  {
    auto m = rotation_matrix();
    cv::Matx33d rmat;
    for (int row = 0; row < 3; row++) {
      for (int col = 0; col < 3; col++) {
        rmat(row, col) = m(row, col);
      }
    }
    cv::Vec3d rvec;
    cv::Rodrigues(rmat, rvec);
    return rvec;
  }

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
  MarkerObservation MarkerObservation::from<std::vector<cv::Point2d>>(
    std::uint64_t id, const std::vector<cv::Point2d> &cfi)
  {
    return MarkerObservation(id, (MarkerObservation::Derived()
      <<
      cfi[0].x, cfi[1].x, cfi[2].x, cfi[3].x,
      cfi[0].y, cfi[1].y, cfi[2].y, cfi[3].y).finished());
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

  template<>
  std::vector<cv::Point3d> Marker::to_corners_f_world<std::vector<cv::Point3d>>(double marker_length) const
  {
    auto corners_f_marker = calc_corners_f_marker(marker_length);
    std::vector<cv::Point3d> corners_f_world{};
    for (int icol = 0; icol < CornersMatrix::ColsAtCompileTime; icol += 1) {
      auto cfw = t_world_marker_.tf() * Translate3{corners_f_marker.col(icol)};
      corners_f_world.emplace_back(cv::Point3d{cfw.t()(0), cfw.t()(1), cfw.t()(2)});
    }
    return corners_f_world;
  }

  using CvCameraCalibration = std::pair<cv::Matx33d, cv::Vec<double, 5>>;

  template<>
  CvCameraCalibration CameraInfo::to<CvCameraCalibration>() const
  {
    cv::Matx33d cm{};
    cv::Vec<double, 5> dc{};
    for (int r = 0; r < 3; r += 1)
      for (int c = 0; c < 3; c += 1) {
        cm(r, c) = camera_matrix_(r, c);
      }
    for (int r = 0; r < DistCoeffs::MaxRowsAtCompileTime; r += 1) {
      dc(r) = dist_coeffs_(r);
    }
    return CvCameraCalibration(cm, dc);
  }

  template<>
  Marker::ProjectFunction Marker::project_t_world_marker<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration,
    const Transform3 &t_world_camera,
    double marker_length)
  {
    return [
      camera_matrix = camera_calibration.first,
      dist_coeffs = camera_calibration.second,
      rvec = t_world_camera.r().to<cv::Vec3d>(),
      tvec = t_world_camera.t().to<cv::Vec3d>(),
      marker_length]
      (const Marker &marker) -> MarkerObservation
    {
      std::vector<cv::Point2d> image_points;
      auto corners_f_world = marker.to_corners_f_world<std::vector<cv::Point3d>>(marker_length);
      cv::projectPoints(corners_f_world, rvec, tvec, camera_matrix, dist_coeffs, image_points);
      return MarkerObservation::from<std::vector<cv::Point2d>>(marker.id(), image_points);
    };
  }

  template<>
  Marker::SolveFunction Marker::solve_t_camera_marker<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration, double marker_length)
  {
    return [
      camera_matrix = camera_calibration.first,
      dist_coeffs = camera_calibration.second,
      marker_length]
      (const MarkerObservation &marker_observation) -> Marker
    {
      // Build up two lists of corner points: 2D in the image frame, 3D in the marker frame.
      auto corners_f_marker{Marker::to_corners_f_marker<std::vector<cv::Point3d>>(marker_length)};
      auto corners_f_image{marker_observation.to<std::vector<cv::Point2d>>()};

      // Figure out marker pose.
      cv::Vec3d rvec, tvec;
      cv::solvePnP(corners_f_marker, corners_f_image,
                   camera_matrix, dist_coeffs,
                   rvec, tvec);

      // rvec, tvec output from solvePnp "brings points from the model coordinate system to the
      // camera coordinate system". In this case the marker frame is the model coordinate system.
      // So rvec, tvec are the transformation t_camera_marker.
      return Marker{marker_observation.id(),
                    Transform3WithCovariance(Transform3(Rotate3::from(rvec), Translate3::from(tvec)))};
    };
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
