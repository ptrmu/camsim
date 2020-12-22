
#include <memory>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker_map.hpp"
#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/calib3d/calib3d.hpp"

namespace fvlam
{

// ==============================================================================
// from fvlam/transform3_with_covariance.cpp
// ==============================================================================

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

// ==============================================================================
// from fvlam/camera_info.cpp
// ==============================================================================

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

// ==============================================================================
// from fvlam/marker_map.cpp
// ==============================================================================

  template<>
  std::vector<cv::Point3d> Marker::to_corners_f_marker<std::vector<cv::Point3d>>(double marker_length)
  {
    auto corners_f_marker = calc_corners3_f_marker(marker_length);
    return std::vector<cv::Point3d>{
      cv::Point3d{corners_f_marker[0].t()(0), corners_f_marker[0].t()(1), corners_f_marker[2].t()(2)},
      cv::Point3d{corners_f_marker[1].t()(0), corners_f_marker[1].t()(1), corners_f_marker[2].t()(2)},
      cv::Point3d{corners_f_marker[2].t()(0), corners_f_marker[2].t()(1), corners_f_marker[2].t()(2)},
      cv::Point3d{corners_f_marker[3].t()(0), corners_f_marker[3].t()(1), corners_f_marker[2].t()(2)}
    };
  }

  template<>
  std::vector<cv::Point3d> Marker::to_corners_f_world<std::vector<cv::Point3d>>(double marker_length) const
  {
    auto corners_f_world = calc_corners3_f_world(marker_length);
    return std::vector<cv::Point3d>{
      cv::Point3d{corners_f_world[0].t()(0), corners_f_world[0].t()(1), corners_f_world[2].t()(2)},
      cv::Point3d{corners_f_world[1].t()(0), corners_f_world[1].t()(1), corners_f_world[2].t()(2)},
      cv::Point3d{corners_f_world[2].t()(0), corners_f_world[2].t()(1), corners_f_world[2].t()(2)},
      cv::Point3d{corners_f_world[3].t()(0), corners_f_world[3].t()(1), corners_f_world[2].t()(2)}
    };
  }

// ==============================================================================
// from fvlam/marker_observation.cpp
// ==============================================================================

  template<>
  std::vector<cv::Point2d> MarkerObservation::to<std::vector<cv::Point2d>>() const
  {
    return std::vector<cv::Point2d>{
      cv::Point2d{corners_f_image_[0].x(), corners_f_image_[0].y()},
      cv::Point2d{corners_f_image_[1].x(), corners_f_image_[1].y()},
      cv::Point2d{corners_f_image_[2].x(), corners_f_image_[2].y()},
      cv::Point2d{corners_f_image_[3].x(), corners_f_image_[3].y()}
    };
  }

  template<>
  MarkerObservation MarkerObservation::from<std::vector<cv::Point2d>>(
    std::uint64_t id, const std::vector<cv::Point2d> &cfi)
  {
    return MarkerObservation(id, MarkerObservation::Array{
      MarkerObservation::Element{cfi[0].x, cfi[0].y},
      MarkerObservation::Element{cfi[1].x, cfi[1].y},
      MarkerObservation::Element{cfi[2].x, cfi[2].y},
      MarkerObservation::Element{cfi[3].x, cfi[3].y}
    });
  }

// ==============================================================================
// fvlam::Marker conversions that require MarkerObservation conversions so
// have to be after those conversions in the file.
// ==============================================================================

  template<>
  Marker::ProjectFunction Marker::project_t_world_marker<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration,
    const Transform3 &t_world_camera,
    double marker_length)
  {
    // gtsam and OpencCv use different frames for their camera coordinates. Our default
    // is the gtsam version os her we have to rotate the camera framw when using the
    // opencv project function
//    auto cv_t_world_camera = Transform3{Rotate3::RzRyRx(0.0, 0.0, 0.0), Translate3{}} * t_world_camera;
    return [
      camera_calibration,
      t_world_camera,
      marker_length]
      (const Marker &marker) -> MarkerObservation
    {
      auto camera_matrix = camera_calibration.first;
      auto dist_coeffs = camera_calibration.second;
      auto rvec = t_world_camera.r().to<cv::Vec3d>();
      auto tvec = t_world_camera.t().to<cv::Vec3d>();

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

  template<>
  Marker::SolveFunction Marker::solve_t_world_marker<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration,
    const Transform3 &t_world_camera,
    double marker_length)
  {
    auto solve_t_camera_marker_function = solve_t_camera_marker<CvCameraCalibration>(camera_calibration,
                                                                                     marker_length);
    return [
      solve_t_camera_marker_function,
      t_world_camera]
      (const MarkerObservation &marker_observation) -> Marker
    {
      auto t_camera_marker = solve_t_camera_marker_function(marker_observation);
      return Marker{marker_observation.id(),
                    Transform3WithCovariance{t_world_camera * t_camera_marker.t_world_marker().tf()}};
    };
  }

  template<>
  Marker::SolveMarkerMarkerFunction Marker::solve_t_marker0_marker1<CvCameraCalibration>(
    const CvCameraCalibration &camera_calibration,
    double marker_length)
  {
    auto solve_t_camera_marker_function = solve_t_camera_marker<CvCameraCalibration>(camera_calibration,
                                                                                     marker_length);
    return [
      solve_t_camera_marker_function,
      marker_length]
      (const MarkerObservation &marker_observation0,
       const MarkerObservation &marker_observation1) -> Transform3WithCovariance
    {
      auto t_camera_marker0 = solve_t_camera_marker_function(marker_observation0).t_world_marker().tf();
      auto t_camera_marker1 = solve_t_camera_marker_function(marker_observation1).t_world_marker().tf();
      return Transform3WithCovariance{t_camera_marker0.inverse() * t_camera_marker1};
    };
  }
}

