
#ifndef _CALIBRATION_BOARD_CONFIG_HPP
#define _CALIBRATION_BOARD_CONFIG_HPP

#include <array>

#include <gtsam/3rdparty/Eigen/Eigen/src/Core/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace camsim
{
  using PointFWorld = gtsam::Point3;
  using PointFBoard = gtsam::Point3;
  using PointFImage = gtsam::Point2;
  using CornerPointsFWorld = std::array<PointFWorld, 4>;
  using CornerPointsFBoard = std::array<PointFBoard, 4>;
  using CornerPointsFImage = std::array<PointFImage, 4>;

  using SquareAddress = Eigen::Vector2i;

// ==============================================================================
// CheckerboardConfig class
// ==============================================================================

  struct CheckerboardConfig
  {
    const bool black_not_white_upper_left_;
    const int squares_x_;
    const int squares_y_;
    const double square_length_;

    CheckerboardConfig(bool black_not_white_upper_left, int squares_x, int squares_y, double square_length) :
      black_not_white_upper_left_{black_not_white_upper_left},
      squares_x_{squares_x},
      squares_y_{squares_y},
      square_length_{square_length}
    {}

    CheckerboardConfig(const CheckerboardConfig &n) :
      black_not_white_upper_left_{n.black_not_white_upper_left_},
      squares_x_{n.squares_x_},
      squares_y_{n.squares_y_},
      square_length_{n.square_length_}
    {}


    // Hold the board so the text reads properly and is at the lower left. The
    // origin of the board's coordinate system is the black corner at the top left.
    // When looking at the board with the origin at the upper left, the x axis is
    // to the right and the y axis is down. The checker board square corners,
    // SquareCorner, are addressed by their ix and iy indices starting with
    // zero at the origin. The physical location of a SquareCorner is calculated
    // as follows:
    PointFBoard to_square_ul_corner_f_board(const SquareAddress &square_address) const
    {
//      assert(square_address(0) >= 0 && square_address.y())
//      return PointFBoard(square_address[0] * square_length_, square_address[1] * square_length_, 0.);
    }
  };

// ==============================================================================
// CharucoboardConfig class
// ==============================================================================

  // Hold the board so the text reads properly and is at the lower left. The
  // origin of the board's coordinate system is the black corner at the top left.
  // When looking at the board with the origin at the upper left, the x axis is
  // to the right and the y axis is down. The checker board square corners,
  // SquareCorner, are addressed by their ix and iy indices starting with
  // zero at the origin. The physical location of a SquareCorner is calculated
  // as follows:
  //
  //  to_square_corner_location(ix, iy)
  //    return (ix * square_length, iy * square_length)
  //
  // The address of a square itself is the same as its upper left corner.  The number
  // of squares horizontally on the board is squares_x and the number of squares
  // vertically on the board is squares_y.
  //
  // A junction is the point where two black squares touch. On a board, there
  // are (squares_x - 1) * (squares_y - 1) junctions. Each has an id, junction_id,
  // that can be calculated as follows:
  //
  //  try_to_junction_id(ix, iy, &junction_id)
  //    if ix <= 0 or ix >= squares_x or  iy <= 0 or iy >= squares_y
  //      return false
  //    junction_id = (squares_x - 1) * iy + (ix - 1)
  //
  //  try_to_square_address(junction_id, &ix, &iy)
  //    if junction_id < 0 or junction_id > (squares_x - 1) * (squares_y - 1)
  //      return false
  //    ix = junction_id % squares_y + 1
  //    iy = junction_id / squares_y + 1
  //
  // Every other square is filled with an aruco marker. Squares
  // where (ix + iy) is odd contain an aruco marker ((0,1), (1,0), (0,3) ...).
  //
  // Every intersection where two black squares intersect has a label.
  // Given ix, iy
  //  no label if ix < 1 or ix > 11
  //  no label if iy < 1 or iy > 8
  //    label = (iy - 1) * 11 + (ix - 1)
  // Given a label
  //  ix = label % 11 + 1
  //  iy = label / 11 + 1
  //
  // Each aruco marker has a tag that is the same as its id
  // Given ix, iy
  //  no tag if ix < 0 or ix > 11
  //  no tag if iy < 0 or iy > 8
  //  no tag if (ix + iy) is even
  // Given tag
  //  iy = tag / 6
  //  ix = (tag % 6) * 2 + (iy is even ? 0 : 1)
  //
  // The corners of an aruco marker are always stored moving clockwise (looking at
  // the marker) around the marker. If looking at the board with the origin at the
  // upper-left, the marker coordinates are stored upper-left, upper-right, lower-right
  // and lower-left.
  // Given the (ix, iy) of a square that contains a marker, the location of the marker
  // corners is:
  //  0: (ix*square_width + (square_width - marker_width) / 2, iy*square_height + (square_height - marker_height) / 2
  //  1: (ix*square_width + (square_width + marker_width) / 2, iy*square_height + (square_height - marker_height) / 2
  //  2: (ix*square_width + (square_width + marker_width) / 2, iy*square_height + (square_height + marker_height) / 2
  //  3: (ix*square_width + (square_width - marker_width) / 2, iy*square_height + (square_height + marker_height) / 2


  struct CharucoboardConfig : public CheckerboardConfig
  {
    const double marker_length_;

    CharucoboardConfig(bool black_not_white_upper_left, int squares_x, int squares_y, double square_length,
                       double marker_length) :
      CheckerboardConfig{black_not_white_upper_left, squares_x, squares_y, square_length},
      marker_length_{marker_length}
    {}

    CharucoboardConfig(const CharucoboardConfig &n) :
      CheckerboardConfig{n},
      marker_length_{n.marker_length_}
    {}
  };

}
#endif //_CALIBRATION_BOARD_CONFIG_HPP
