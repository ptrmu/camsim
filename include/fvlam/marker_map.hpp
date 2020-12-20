
#ifndef FVLAM_MARKER_MAP_HPP
#define FVLAM_MARKER_MAP_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <array>
#include <map>
#include <vector>

#include "fvlam/marker_observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include <Eigen/Geometry>

namespace fvlam
{
// ==============================================================================
// Marker class
// ==============================================================================

  class Marker
  {
  public:
    using CornersMatrix = Eigen::Matrix<double, 3, 4>;

  private:
    // The id of the marker
    std::uint64_t id_{};

    // The pose of the marker in some world frame. Which frame is world depends on the context.
    Transform3WithCovariance t_world_marker_;

    // Prevent modification if true
    bool is_fixed_{false};

     inline static CornersMatrix unit_corners_f_marker()
    {
      return (CornersMatrix() << -0.5, 0.5, 0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0, 0, 0, 0).finished();
    }

    inline static CornersMatrix calc_corners_f_marker(double marker_length)
    {
      return unit_corners_f_marker() * marker_length;
    }

    inline CornersMatrix calc_corners_f_world(double marker_length) const
    {
      auto corners_f_marker = calc_corners_f_marker(marker_length);
      return (CornersMatrix()
        <<
        (t_world_marker_.tf() * Translate3{corners_f_marker.block<3, 1>(0, 0)}).mu(),
        (t_world_marker_.tf() * Translate3{corners_f_marker.block<3, 1>(0, 1)}).mu(),
        (t_world_marker_.tf() * Translate3{corners_f_marker.block<3, 1>(0, 2)}).mu(),
        (t_world_marker_.tf() * Translate3{corners_f_marker.block<3, 1>(0, 3)}).mu()).finished();
    }

  public:
    Marker() = default;

    Marker(std::uint64_t id, Transform3WithCovariance t_world_marker, bool is_fixed = false) :
      id_(id), t_world_marker_(std::move(t_world_marker)), is_fixed_(is_fixed)
    {}

    auto id() const
    { return id_; }

    auto is_fixed() const
    { return is_fixed_; }

    void set_is_fixed(bool is_fixed)
    { is_fixed_ = is_fixed; }

    const auto &t_world_marker() const
    { return t_world_marker_; }

    template<typename T>
    static Marker from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const; //
    std::string to_id_string() const; //
    std::string to_corners_f_world_string(double marker_length) const; //

    template<typename T>
    T to_corners_f_world(double marker_length) const;

    template<typename T>
    static T to_corners_f_marker(double marker_length);

    using ProjectFunction = std::function<MarkerObservation(const Marker &marker)>;
    using SolveFunction = std::function<Marker(const MarkerObservation &observation)>;

    template<typename TCameraCalibration>
    static ProjectFunction project_t_world_marker(const TCameraCalibration &camera_calibration,
                                                  const Transform3 &t_world_camera,
                                                  double marker_length);

    template<typename TCameraCalibration>
    static ProjectFunction project_t_camera_marker(const TCameraCalibration &camera_calibration,
                                                   double marker_length)
    {
      return project_t_world_marker<TCameraCalibration>(camera_calibration, Transform3{}, marker_length);
    }

    template<typename TCameraCalibration>
    static SolveFunction solve_t_camera_marker(const TCameraCalibration &camera_calibration,
                                               double marker_length);
  };

// ==============================================================================
// MarkerMap class
// ==============================================================================

  class MarkerMap
  {
    const double marker_length_;
    std::map<std::uint64_t, Marker> markers_;

  public:
    MarkerMap() = delete;

    explicit MarkerMap(double marker_length) :
      marker_length_{marker_length}, markers_{}
    {}

    const auto &markers() const
    { return markers_; }

    auto marker_length() const
    { return marker_length_; }

    template<typename T>
    static Translate3 from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;

    Marker *find_marker(int id);

    const Marker *find_marker_const(std::uint64_t id) const;

    void add_marker(Marker marker);
  };
}
#endif //FVLAM_MARKER_MAP_HPP
