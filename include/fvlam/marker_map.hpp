
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
    static constexpr size_t ArraySize = 4;
    using Element2 = Translate2;
    using Element3 = Translate3;
    using Array2 = std::array<Element2, ArraySize>;
    using Array3 = std::array<Element3, ArraySize>;
    using Mu2Vector = Eigen::Matrix<double, Element2::MuVector::MaxRowsAtCompileTime * ArraySize, 1>;
    using Mu3Vector = Eigen::Matrix<double, Element3::MuVector::MaxRowsAtCompileTime * ArraySize, 1>;

  private:
    // The id of the marker
    std::uint64_t id_{};

    // The pose of the marker in some world frame. Which frame is world depends on the context.
    Transform3WithCovariance t_world_marker_;

    // Prevent modification if true
    bool is_fixed_{false};

    inline static Array2 unit_corners2_f_marker()
    {
      return Array2{Translate2{-1, 1},
                    Translate2{1, 1},
                    Translate2{1, -1},
                    Translate2{-1, -1}};
    }

    inline static Array3 unit_corners3_f_marker()
    {
      return Array3{Translate3{-1, 1, 0},
                    Translate3{1, 1, 0},
                    Translate3{1, -1, 0},
                    Translate3{-1, -1, 0}};
    }

    inline static Array2 calc_corners2_f_marker(double marker_length)
    {
      auto half_marker_length{marker_length / 2.0};
      auto unit_corners2{unit_corners2_f_marker()};
      return Array2{unit_corners2[0] * half_marker_length,
                    unit_corners2[1] * half_marker_length,
                    unit_corners2[2] * half_marker_length,
                    unit_corners2[3] * half_marker_length};
    }

    inline static Array3 calc_corners3_f_marker(double marker_length)
    {
      auto half_marker_length{marker_length / 2.0};
      auto unit_corners3{unit_corners3_f_marker()};
      return Array3{unit_corners3[0] * half_marker_length,
                    unit_corners3[1] * half_marker_length,
                    unit_corners3[2] * half_marker_length,
                    unit_corners3[3] * half_marker_length};
    }

    inline Array3 calc_corners3_f_world(double marker_length) const
    {
      auto corners_f_marker = calc_corners3_f_marker(marker_length);
      return Array3{t_world_marker_.tf() * corners_f_marker[0],
                    t_world_marker_.tf() * corners_f_marker[1],
                    t_world_marker_.tf() * corners_f_marker[2],
                    t_world_marker_.tf() * corners_f_marker[3]};
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

    template<class T>
    static Marker from(const T &other);

    template<class T>
    T to() const;

    std::string to_string(bool also_cov = false) const; //
    std::string to_id_string() const; //
    std::string to_corners_f_world_string(double marker_length) const; //

    template<class T>
    T to_corners_f_world(double marker_length) const;

    template<class T>
    static T to_corners_f_marker(double marker_length);

    using ProjectFunction = std::function<MarkerObservation(const Marker &marker)>;
    using SolveFunction = std::function<Marker(const MarkerObservation &observation)>;
    using SolveMarkerMarkerFunction = std::function<Transform3WithCovariance(const MarkerObservation &observation0,
                                                                             const MarkerObservation &observation1)>;

    template<class TCameraCalibration>
    static ProjectFunction project_t_world_marker(const TCameraCalibration &camera_calibration,
                                                  const Transform3 &t_world_camera,
                                                  double marker_length);

    template<class TCameraCalibration>
    static ProjectFunction project_t_camera_marker(const TCameraCalibration &camera_calibration,
                                                   double marker_length)
    {
      return project_t_world_marker<TCameraCalibration>(camera_calibration, Transform3{}, marker_length);
    }

    template<class TCameraCalibration>
    static SolveFunction solve_t_camera_marker(const TCameraCalibration &camera_calibration,
                                               double marker_length);

    template<class TCameraCalibration>
    static SolveFunction solve_t_world_marker(const TCameraCalibration &camera_calibration,
                                              const Transform3 &t_world_camera,
                                              double marker_length);

    template<class TCameraCalibration>
    static SolveMarkerMarkerFunction solve_t_marker0_marker1(const TCameraCalibration &camera_calibration,
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

    template<class T>
    static Translate3 from(const T &other);

    template<class T>
    T to() const;

    std::string to_string(bool also_cov = false) const;

    Marker *find_marker(int id);

    const Marker *find_marker_const(std::uint64_t id) const;

    void add_marker(Marker marker)
    {
      markers_.emplace(marker.id(), std::move(marker));
    }
  };
}
#endif //FVLAM_MARKER_MAP_HPP
