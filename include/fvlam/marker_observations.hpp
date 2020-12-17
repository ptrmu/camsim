#ifndef FVLAM_OBSERVATIONS_HPP
#define FVLAM_OBSERVATIONS_HPP
#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include <vector>

namespace fvlam
{
// ==============================================================================
// Observation class
// ==============================================================================

  class MarkerObservation
  {
  public:
    using Derived = Eigen::Matrix<double, 2, 4>;
    using MuVector = Eigen::Matrix<double, Derived::MaxRowsAtCompileTime, 1>;

    // The id of the marker that we observed.
    std::uint64_t id_{0};

    // The four marker corners in the image frame.
    // origin = upper left, x -> left to right, y -> top to bottom
    Derived corners_f_image_{Derived::Zero()};

  public:
    MarkerObservation(std::uint64_t id, Derived corners_f_image)
      : id_(id), corners_f_image_(std::move(corners_f_image))
    {}

    MarkerObservation(std::uint64_t id,
                      double x0, double y0,
                      double x1, double y1,
                      double x2, double y2,
                      double x3, double y3)
      : id_(id), corners_f_image_((Derived{} << x0, x1, x2, x3, y0, y1, y2, y3).finished())
    {}

    auto id() const
    { return id_; }

    auto &corners_f_image() const
    { return corners_f_image_; }

    template<typename T>
    static MarkerObservation from(const T &other);

    template<typename T>
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
