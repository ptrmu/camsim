#pragma once
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <array>
#include <map>
#include <vector>

#include <Eigen/Geometry>
#include "observation.hpp"
#include "transform3_with_covariance.hpp"

namespace fvlam
{
  class Logger;

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

  private:
    // The id of the marker
    std::uint64_t id_;

    // The pose of the marker in some world frame. Which frame is world depends on the context.
    Transform3WithCovariance t_map_marker_;

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

  public:
    inline Array3 calc_corners3_f_world(double marker_length) const
    {
      auto corners_f_marker = calc_corners3_f_marker(marker_length);
      return Array3{t_map_marker_.tf() * corners_f_marker[0],
                    t_map_marker_.tf() * corners_f_marker[1],
                    t_map_marker_.tf() * corners_f_marker[2],
                    t_map_marker_.tf() * corners_f_marker[3]};
    }

  public:
   Marker() :
      id_(0), t_map_marker_(), is_fixed_(false)
    {}

    Marker(std::uint64_t id, Transform3WithCovariance t_map_marker, bool is_fixed = false) :
      id_(id), t_map_marker_(std::move(t_map_marker)), is_fixed_(is_fixed)
    {}

    auto is_valid() const
    { return t_map_marker_.is_valid(); }

    auto id() const
    { return id_; }

    auto is_fixed() const
    { return is_fixed_; }

    const auto &t_map_marker() const
    { return t_map_marker_; }

    template<class T>
    static Marker from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    // Return the marker corners in the marker frame (z=0)
    template<class T>
    static T corners_f_marker(double marker_length);

    // Return the marker corners in the world frame given the t_world_marker transform.
    template<class T>
    T corners_f_world(double marker_length) const; //
    template<class T>
    void corners_f_world(double marker_length, T &other) const;

    std::string to_string(bool also_cov = false) const; //
    std::string to_id_string() const; //
    std::string to_corners_f_world_string(double marker_length) const; //

    bool equals(const Marker &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    using ProjectFunction = std::function<Observation(const Marker &marker)>;

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
  };

// ==============================================================================
// MapEnvironment class
// ==============================================================================

  class MapEnvironment
  {
    std::string description_;
    int marker_dictionary_id_;
    double marker_length_;

  public:
    MapEnvironment() :
      description_{}, marker_dictionary_id_{0}, marker_length_{0.0}
    {}

    MapEnvironment(std::string description,
                   int marker_dictionary_id,
                   double marker_length) :
      description_{std::move(description)},
      marker_dictionary_id_{marker_dictionary_id},
      marker_length_{marker_length}
    {}

    const auto &description() const
    { return description_; }

    const auto &marker_dictionary_id() const
    { return marker_dictionary_id_; }

    const auto &marker_length() const
    { return marker_length_; }

    template<class T>
    static MapEnvironment from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string() const;

    bool equals(const MapEnvironment &other, double tol = 1.0e-9, bool check_relative_also = true) const;
  };

// ==============================================================================
// MarkerMap class
// ==============================================================================

  class MarkerMap
  {
    MapEnvironment map_environment_;
    std::map<std::uint64_t, Marker> m_{};

  public:
    explicit MarkerMap() :
      map_environment_{}
    {}

    explicit MarkerMap(MapEnvironment map_environment) :
      map_environment_{std::move(map_environment)}
    {}

    const auto &map_environment() const
    { return map_environment_; }

    auto marker_length() const
    { return map_environment_.marker_length(); }

    auto &m_mutable()
    { return m_; }

    auto &m() const
    { return m_; }

    auto size() const
    { return m_.size(); }

    auto empty() const
    { return m_.empty(); }

    template<class T>
    static MarkerMap from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    std::string to_string(bool also_cov = false) const;

    bool equals(const MarkerMap &other, double tol = 1.0e-9, bool check_relative_also = true) const;

    Marker *find_marker(int id)
    {
      auto marker_pair = m_.find(id);
      return marker_pair == m_.end() ? nullptr : &marker_pair->second;
    }

    const Marker *find_marker_const(std::uint64_t id) const
    {
      auto marker_pair = m_.find(id);
      return marker_pair == m_.end() ? nullptr : &marker_pair->second;
    }

    void add_marker(Marker marker)
    {
      m_.emplace(marker.id(), std::move(marker));
    }

    void save(const std::string &filename, Logger &logger) const; //
    static MarkerMap load(const std::string &filename, Logger &logger); //
  };
}
