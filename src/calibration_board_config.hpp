
#ifndef _CALIBRATION_BOARD_CONFIG_HPP
#define _CALIBRATION_BOARD_CONFIG_HPP

#include <array>

//#include <gtsam/3rdparty/Eigen/Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  using PointFFacade = gtsam::Vector2; // the surface that contains markings
  using PointFBoard = gtsam::Point3;
  using PointFWorld = gtsam::Point3;
  using PointFImage = gtsam::Vector2;
  using CornerPointsFFacade = gtsam::Matrix24;
  using CornerPointsFBoard = gtsam::Matrix34;
  using CornerPointsFWorld = gtsam::Matrix34;
  using CornerPointsFImage = gtsam::Matrix24;

  using SquareAddress = Eigen::Vector2i;
  using SquareId = std::uint64_t;
  using JunctionId = std::uint64_t;
  using ArucoId = std::uint64_t;

// ==============================================================================
// CheckerboardConfig class
// ==============================================================================

  struct CheckerboardConfig
  {
    const std::uint64_t squares_x_;
    const std::uint64_t squares_y_;
    const double square_length_;

    const std::uint64_t max_square_id_;
    const double square_length_half_;

  private:
    const std::uint64_t squares_x_m_1_;
    const std::uint64_t squares_y_m_1_;

  public:
    const double board_width_half_;
    const double board_height_half_;
    const std::uint64_t max_junction_id_;

    CheckerboardConfig(std::uint64_t squares_x, std::uint64_t squares_y, double square_length) :
      squares_x_{squares_x},
      squares_y_{squares_y},
      square_length_{square_length},
      max_square_id_{squares_x * squares_y},
      square_length_half_{square_length / 2},
      squares_x_m_1_{squares_x_ - 1}, squares_y_m_1_{squares_y_ - 1},
      max_junction_id_{squares_x_m_1_ * squares_y_m_1_},
      board_width_half_{squares_x_ * square_length_ / 2.},
      board_height_half_{squares_y_ * square_length_ / 2.}
    {}

    CheckerboardConfig(const CheckerboardConfig &) = default;               // Copy constructor
    CheckerboardConfig(CheckerboardConfig &&) = default;                    // Move constructor
//    CheckerboardConfig &operator=(const CheckerboardConfig &) = default;    // Copy assignment operator
//    CheckerboardConfig &operator=(CheckerboardConfig &&) = default;         // Move assignment operator
    ~CheckerboardConfig() = default;                                        // Destructor

    // Hold the board so the text reads properly and is at the lower left. The
    // origin of the board's coordinate system is the black corner at the top left.
    // When looking at the board with the origin at the upper left, the x axis is
    // to the right and the y axis is down. The checker board squares
    // are addressed by their ix and iy indices starting with
    // zero at the origin - SquareAddress. Each square has an ID - SquareId.
  private:
    SquareId to_square_id_(const SquareAddress &square_address) const
    {
      return square_address.y() * squares_x_ + square_address.x();
    } //
    SquareAddress to_square_address_(SquareId square_id) const
    {
      return SquareAddress{square_id % squares_x_, square_id / squares_x_};
    } //
  public:
    SquareId to_square_id(const SquareAddress &square_address) const
    {
      assert(square_address.x() >= 0 && square_address.x() < squares_x_ &&
             square_address.y() >= 0 && square_address.y() < squares_y_);
      return to_square_id_(square_address);
    } //
    SquareAddress to_square_address(SquareId square_id) const
    {
      assert(square_id <= max_square_id_);
      return to_square_address_(square_id);
    }

    // The location of each square is the center of the square. The square_point
    // is the upper left corner of the square.
    PointFFacade to_square_point(const SquareAddress &square_address) const
    {
      auto x(square_address.x() * square_length_);
      auto y(square_address.y() * square_length_);
      return PointFFacade{x, y};
    } //
    PointFFacade to_square_location_(const SquareAddress &square_address) const
    {
      return to_square_point(square_address).array() + 0.5;
    } //
    PointFFacade to_square_location(const SquareAddress &square_address) const
    {
      assert(square_address.x() >= 0 && square_address.x() < squares_x_ &&
             square_address.y() >= 0 && square_address.y() < squares_y_);
      return to_square_location_(square_address);
    }

    // A junction is the point where two black squares touch. On a board, there
    // are (squares_x - 1) * (squares_y - 1) junctions.
    PointFFacade to_junction_location(JunctionId junction_id) const
    {
      assert(junction_id >= 0 && junction_id < max_junction_id_);
      auto x(junction_id % squares_x_m_1_ + 1);
      auto y(junction_id / squares_x_m_1_ + 1);
      return to_square_point(SquareAddress(x, y));
    }//

    PointFBoard to_point_f_board(PointFFacade point_f_facade) const
    {
      auto x(point_f_facade.x() - board_width_half_);
      auto y(-(point_f_facade.y() - board_height_half_)); // facade y is down, board y is up
      return PointFBoard{x, y, 0.0};
    }
  };

// ==============================================================================
// CharucoboardConfig class
// ==============================================================================

  struct CharucoboardConfig : public CheckerboardConfig
  {
    const std::uint64_t upper_left_white_not_black_;
    const double marker_length_;

  private:
    const double offset_to_aruco_;
    const std::uint64_t squares_x_odd_;
    const std::uint64_t squares_y_odd_;
    const std::uint64_t arucos_on_even_row_;
    const std::uint64_t arucos_on_odd_row_;

  public:
    const double marker_length_half_;
    const std::uint64_t max_aruco_id_;

    CharucoboardConfig(std::uint64_t squares_x, std::uint64_t squares_y, double square_length,
                       bool upper_left_white_not_black, double marker_length) :
      CheckerboardConfig{squares_x, squares_y, square_length},
      upper_left_white_not_black_(upper_left_white_not_black ? 1 : 0),
      marker_length_{marker_length},
      offset_to_aruco_{(square_length - marker_length) / 2},
      squares_x_odd_{squares_x & 1U},
      squares_y_odd_{squares_y & 1U},
      arucos_on_even_row_{squares_x / 2 + (upper_left_white_not_black_ & squares_x_odd_)},
      arucos_on_odd_row_{squares_x - arucos_on_even_row_},
      marker_length_half_{marker_length / 2.0},
      max_aruco_id_{squares_y / 2 * squares_x + squares_y_odd_ * arucos_on_even_row_}
    {}

    CharucoboardConfig(const CharucoboardConfig &) = default;               // Copy constructor
    CharucoboardConfig(CharucoboardConfig &&) = default;                    // Move constructor
//    CharucoboardConfig &operator=(const CharucoboardConfig &) = default;    // Copy assignment operator
//    CharucoboardConfig &operator=(CharucoboardConfig &&) = default;         // Move assignment operator
    ~CharucoboardConfig() = default;                                        // Destructor

    // An aruco symbol is situated in all of the white squares on the board. It happens
    // that the aruco tag is the same as its id. The Aruco Location is the center of the
    // white square that contains the symbol.
    PointFFacade to_aruco_location(ArucoId aruco_id) const
    {
      assert(aruco_id >= 0 && aruco_id < max_aruco_id_);
      std::uint64_t x_group = aruco_id % squares_x_;
      std::uint64_t y_group = aruco_id / squares_x_;
      std::uint64_t odd_row = (x_group >= arucos_on_even_row_) ? 1U : 0U;
      std::uint64_t ix = (x_group - (odd_row * arucos_on_even_row_)) * 2 + (1 ^ odd_row ^ upper_left_white_not_black_);
      std::uint64_t iy = y_group * 2 + odd_row;
      return to_square_point(SquareAddress(ix, iy)).array() + square_length_half_;
    }//

    // Returns the location of the corners relative to the center of the aruco marker,
    // The corners of an aruco marker are always stored moving clockwise (looking at
    // the marker) around the marker. If looking at the board with the origin at the
    // upper-left, the marker coordinates are stored upper-left, upper-right, lower-right
    // and lower-left.
    // Note that the y axis points down the board but we would like the marker to be
    // positioned upright (its y axis points up the board).
    CornerPointsFFacade to_aruco_corners_f_marker() const
    {
      return (CornerPointsFFacade{} << PointFFacade{-marker_length_half_, -marker_length_half_},
        PointFFacade{marker_length_half_, -marker_length_half_},
        PointFFacade{marker_length_half_, marker_length_half_},
        PointFFacade{-marker_length_half_, marker_length_half_}).finished();
    };

    // Returns the location of the corners relative to the markings in 2D (origin=board top left)
    CornerPointsFFacade to_aruco_corners_f_facade(ArucoId aruco_id) const
    {
      return to_aruco_corners_f_marker() + to_aruco_location(aruco_id).replicate<1, 4>();
    }

    // Returns the location of the corners relative to the markings in 3D (z = 0, origin=board center)
    CornerPointsFBoard to_aruco_corners_f_board(const CornerPointsFFacade &corners_f_facade) const
    {
      auto temp_corners_f_facade{corners_f_facade};
      // note: facade y is down, board y is up
      temp_corners_f_facade.row(0) = temp_corners_f_facade.row(0).array() - board_width_half_;
      temp_corners_f_facade.row(1) = -(temp_corners_f_facade.row(1).array() - board_height_half_);
      return (CornerPointsFBoard{} << temp_corners_f_facade, gtsam::Matrix14{}.setZero()).finished();
    }
  };
}
#endif //_CALIBRATION_BOARD_CONFIG_HPP
