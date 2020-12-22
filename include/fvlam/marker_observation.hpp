#ifndef FVLAM_OBSERVATIONS_HPP
#define FVLAM_OBSERVATIONS_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <vector>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{
// ==============================================================================
// Observation class
// ==============================================================================

  class MarkerObservation
  {
  public:
    static constexpr size_t ArraySize = 4;
    using Element = Translate2;
    using Array = std::array<Element, ArraySize>;
    using MuVector = Eigen::Matrix<double, Element::MuVector::MaxRowsAtCompileTime * ArraySize, 1>;

    // The id of the marker that we observed.
    std::uint64_t id_;

    // The four marker corners in the image frame.
    // origin = upper left, x -> left to right, y -> top to bottom
    Array corners_f_image_;

  public:
    MarkerObservation() :
      id_{0}, corners_f_image_{Translate2(), Translate2(), Translate2(), Translate2()}
    {}

    MarkerObservation(std::uint64_t id, Array corners_f_image)
      : id_(id), corners_f_image_(std::move(corners_f_image))
    {}

    MarkerObservation(std::uint64_t id,
                      double x0, double y0,
                      double x1, double y1,
                      double x2, double y2,
                      double x3, double y3)
      : id_(id), corners_f_image_{Element{x0, y0},
                                  Element{x1, y1},
                                  Element{x2, y2},
                                  Element{x3, y3}}
    {}

    auto id() const
    { return id_; }

    auto &corners_f_image() const
    { return corners_f_image_; }

    template<class T>
    static MarkerObservation from(const T &other);

    template<class T>
    static MarkerObservation from(std::uint64_t id, const T &other);

    template<class T>
    T to() const;

    std::string to_string() const;
  };

// ==============================================================================
// Observations class
// ==============================================================================

  class MarkerObservations
  {
    std::uint64_t stamp_;

    // The list of observations
    std::vector<MarkerObservation> observations_{};

  public:
    explicit MarkerObservations(std::uint64_t stamp = 0) :
      stamp_(stamp)
    {}

    auto const &observations() const
    { return observations_; }

    auto size() const
    { return observations_.size(); }

    template<typename T>
    static MarkerObservations from(const T &other);

    template<typename T>
    T to() const;

    void add(const MarkerObservation &observation)
    {
      observations_.emplace_back(observation);
    }
  };


}
#endif // FVLAM_OBSERVATIONS_HPP
