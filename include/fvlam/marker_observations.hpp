#ifndef FVLAM_OBSERVATIONS_HPP
#define FVLAM_OBSERVATIONS_HPP

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
      : id_(id), corners_f_image_(corners_f_image)
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
    // The list of observations
    std::vector<MarkerObservation> observations_{};

  public:
    auto &observations() const
    { return observations_; }

    auto size() const
    { return observations_.size(); }

    void add(const MarkerObservation &observation); //
  };


}
#endif // FVLAM_OBSERVATIONS_HPP
