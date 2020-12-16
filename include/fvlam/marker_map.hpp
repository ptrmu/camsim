
#ifndef FVLAM_MARKER_MAP_HPP
#define FVLAM_MARKER_MAP_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"

#include <array>
#include <map>
#include <vector>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{
// ==============================================================================
// Marker class
// ==============================================================================

  class Marker
  {
    // The id of the marker
    std::uint64_t id_{};

    // The pose of the marker in the map frame
    Transform3WithCovariance t_map_marker_;

    // Prevent modification if true
    bool is_fixed_{false};

  public:
    Marker() = default;

    Marker(std::uint64_t id, Transform3WithCovariance t_map_marker, bool is_fixed = false) :
      id_(id), t_map_marker_(std::move(t_map_marker)), is_fixed_(is_fixed)
    {}

    auto id() const
    { return id_; }

    auto is_fixed() const
    { return is_fixed_; }

    void set_is_fixed(bool is_fixed)
    { is_fixed_ = is_fixed; }

    const auto &t_map_marker() const
    { return t_map_marker_; }

    template<typename T>
    static Translate3 from(const T &other);

    template<typename T>
    T to() const;

    std::string to_string() const;

    std::array<Translate3, 4> corners_f_map(double marker_length) const;
  };

// ==============================================================================
// MarkerMap class
// ==============================================================================

  class MarkerMap
  {
    const double marker_length_;
    std::map<std::uint64_t, Marker> markers_{};

  public:
    MarkerMap() = delete;

    explicit MarkerMap(double marker_length_);

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
