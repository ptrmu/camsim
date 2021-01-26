#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <memory>

#include "transform3_with_covariance.hpp"

namespace rclcpp
{
  class Time;
}

namespace cv
{
  class Mat; //
}

namespace fvlam
{
  class Logger; //
  class CameraInfo; //
  class MarkerMap; //
  class Observation; //
  class Observations; //

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

// An interface used to localize a camera from fiducial marker observations.
  class LocalizeCameraInterface
  {
  public:
    virtual ~LocalizeCameraInterface() = default;

    // Given observations of fiducial markers and a map of world locations of those
    // markers, figure out the camera pose in the world frame.
    virtual Transform3WithCovariance solve_t_map_camera(const Observations &observations,
                                                        const CameraInfo &camera_info,
                                                        const MarkerMap &map) = 0;

    // Given the corners of one marker (observation) calculate t_camera_marker.
    virtual Transform3WithCovariance solve_t_camera_marker(const Observation &observation,
                                                           const CameraInfo &camera_info,
                                                           double marker_length) = 0;
  };

  template<class TLcContext>
  std::unique_ptr<LocalizeCameraInterface> make_localize_camera(const TLcContext &lc_context,
                                                                Logger &logger);

// ==============================================================================
// LocalizeCameraCvContext class
// ==============================================================================

  struct LocalizeCameraCvContext
  {

    template<class T>
    static LocalizeCameraCvContext from(T &other);
  };

// ==============================================================================
// LocalizeCameraProjectBetweenContext class
// ==============================================================================

  struct LocalizeCameraProjectBetweenContext
  {
    double &corner_measurement_sigma_;
    bool &use_marker_covariance_;

    explicit LocalizeCameraProjectBetweenContext(double &corner_measurement_sigma,
                                        bool &use_marker_covariance) :
      corner_measurement_sigma_{corner_measurement_sigma},
      use_marker_covariance_{use_marker_covariance}
    {}

    template<class T>
    static LocalizeCameraProjectBetweenContext from(T &other);
  };

// ==============================================================================
// LocalizeCameraResectioningContext class
// ==============================================================================

  struct LocalizeCameraResectioningContext
  {
    double &corner_measurement_sigma_;

    explicit LocalizeCameraResectioningContext(double &corner_measurement_sigma) :
      corner_measurement_sigma_{corner_measurement_sigma}
    {}

    template<class T>
    static LocalizeCameraResectioningContext from(T &other);
  };


// ==============================================================================
// FiducialMarkerInterface class
// ==============================================================================

// An interface used to deal with fiducial markers.
  class FiducialMarkerInterface
  {
  public:
    virtual ~FiducialMarkerInterface() = default;

    // Look for fiducial markers in a gray image.
    virtual Observations detect_markers(cv::Mat &gray_image) = 0;

    // Draw the boundary around detected markers.
    virtual void annotate_image_with_detected_markers(cv::Mat &color_image,
                                                      const Observations &observations) = 0;

    // Draw axes on a marker in an image.
    virtual void annotate_image_with_marker_axis(cv::Mat &color_image,
                                                 const Transform3 &t_camera_marker,
                                                 const CameraInfo &camera_info,
                                                 double axis_length) = 0;
  };

  template<class TFmContext>
  std::unique_ptr<FiducialMarkerInterface> make_fiducial_marker(const TFmContext &fm_context,
                                                                Logger &logger);

// ==============================================================================
// LocalizeCameraCvContext class
// ==============================================================================

  struct FiducialMarkerCvContext
  {
    double border_color_red_{0.0};
    double border_color_green_{1.0};
    double border_color_blue_{0.0};
    int aruco_dictionary_id_{0};
    int &corner_refinement_method_;

    explicit FiducialMarkerCvContext(int &corner_refinement_method) :
      corner_refinement_method_{corner_refinement_method}
    {}

    template<class T>
    static FiducialMarkerCvContext from(T &other);
  };
}