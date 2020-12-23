
#include "catch2/catch.hpp"
#include "../cal/cal_run.hpp"
#include "../cal/cal_solver_runner.hpp"
#include "../calibration_model.hpp"
#include "../model.hpp"

namespace camsim
{
#if 0

  TEST_CASE("aruco locations")
  {
    SECTION("3x3 black") {
      CharucoboardConfig ar_cfg(3, 3, 12, false, 6);
      REQUIRE(ar_cfg.junction_id_to_junction_location(0).x() == 12.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(1).x() == 24.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(2).x() == 12.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(3).x() == 24.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(0).y() == 12.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(1).y() == 12.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(2).y() == 24.);
      REQUIRE(ar_cfg.junction_id_to_junction_location(3).y() == 24.);

      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(0)).x() == -6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(1)).x() == 6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(2)).x() == -6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(3)).x() == 6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(0)).y() == 6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(1)).y() == 6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(2)).y() == -6.);
      REQUIRE(ar_cfg.to_point_f_board(ar_cfg.junction_id_to_junction_location(3)).y() == -6.);

      REQUIRE(ar_cfg.to_aruco_location(0).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 30.);

      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 0) == -3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 1) == 3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 2) == 3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 3) == -3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 0) == 15.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 1) == 15.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 2) == 9.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 3) == 9.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 3);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 1);
    }

    SECTION("3x4 black") {
      CharucoboardConfig ar_cfg(3, 4, 12, false, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(4).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(5).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(4).y() == 42.);
      REQUIRE(ar_cfg.to_aruco_location(5).y() == 42.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 3);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 1);
    }

    SECTION("4x3 black") {
      CharucoboardConfig ar_cfg(4, 3, 12, false, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 42.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(4).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(5).x() == 42.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(4).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(5).y() == 30.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 4);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 0);
    }

    SECTION("3x3 white") {
      CharucoboardConfig ar_cfg(3, 3, 12, true, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(4).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(4).y() == 30.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 4);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 0);
    }

    SECTION("3x4 white") {
      CharucoboardConfig ar_cfg(3, 4, 12, true, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(4).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(5).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(4).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(5).y() == 42.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 4);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 0);
    }

    SECTION("4x3 white") {
      CharucoboardConfig ar_cfg(4, 3, 12, true, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 42.);
      REQUIRE(ar_cfg.to_aruco_location(4).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(5).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(4).y() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(5).y() == 30.);

      REQUIRE(ar_cfg.get_adjacent_arucos(0)[0] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos(0)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(1)[1] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[0] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos(2)[1] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos(3)[1] == 4);

      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(0)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(1)[1] == 1);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[0] == 2);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(2)[1] == 0);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[0] == 3);
      REQUIRE(ar_cfg.get_adjacent_arucos_closest_corner(3)[1] == 1);
    }
  }

  TEST_CASE("Charucoboard Model generic generation")
  {
    auto equals{gtsam::equals<double>{}};

    ModelConfig model_config{[]() -> std::vector<gtsam::Pose3>
                             {
                               return std::vector<gtsam::Pose3>{gtsam::Pose3{gtsam::Rot3::RzRyRx(0., 0., 0.),
                                                                             gtsam::Point3{0., 0., -2.}}};
                             },
                             []() -> std::vector<gtsam::Pose3>
                             {
                               return std::vector<gtsam::Pose3>{gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI, 0., M_PI),
                                                                             gtsam::Point3{0., 0., 0.}}};
                             },
                             camsim::CameraTypes::simple_camera,
                             0.1775};

    SECTION("3x4 black") {
      int squares_x = 3;
      int squares_y = 4;
      double square_length = 12.0;
      double marker_length = 6.0;

      CharucoboardConfig ar_cfg(squares_x, squares_y, square_length, false, marker_length);

      CharucoboardCalibrationModel acm(model_config, ar_cfg);

      REQUIRE(acm.boards_.boards_.size() == 1);

      REQUIRE(acm.boards_.junctions_f_board_.size() == 6);
      REQUIRE(acm.boards_.junctions_f_board_[0].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[1].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[2].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[3].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[4].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[5].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[0].y() == -(1 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[1].y() == -(1 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[2].y() == -(2 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[3].y() == -(2 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[4].y() == -(3 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[5].y() == -(3 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.junctions_f_board_[0].z() == 0.0);
      REQUIRE(acm.boards_.junctions_f_board_[1].z() == 0.0);
      REQUIRE(acm.boards_.junctions_f_board_[2].z() == 0.0);
      REQUIRE(acm.boards_.junctions_f_board_[3].z() == 0.0);
      REQUIRE(acm.boards_.junctions_f_board_[4].z() == 0.0);
      REQUIRE(acm.boards_.junctions_f_board_[5].z() == 0.0);

      REQUIRE(acm.boards_.boards_[0].junctions_f_world_.size() == 6);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[0].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[1].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[2].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[3].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[4].x() == 1 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[5].x() == 2 * square_length - squares_x * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[0].y() == -(1 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[1].y() == -(1 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[2].y() == -(2 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[3].y() == -(2 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[4].y() == -(3 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[5].y() == -(3 * square_length - squares_y * square_length / 2));
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[0].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[1].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[2].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[3].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[4].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[5].z() == -2.0);

      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](1, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](0, 3) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](1, 3) == -9.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);

      REQUIRE(acm.junctions_f_images_[0][0][0].junction_(0) == 47.);
      REQUIRE(acm.junctions_f_images_[0][0][1].junction_(0) == 53.);
      REQUIRE(acm.junctions_f_images_[0][0][2].junction_(0) == 47.);
      REQUIRE(acm.junctions_f_images_[0][0][3].junction_(0) == 53.);
      REQUIRE(acm.junctions_f_images_[0][0][4].junction_(0) == 47.);
      REQUIRE(acm.junctions_f_images_[0][0][5].junction_(0) == 53.);
      REQUIRE(acm.junctions_f_images_[0][0][0].junction_(1) == 44.);
      REQUIRE(acm.junctions_f_images_[0][0][1].junction_(1) == 44.);
      REQUIRE(acm.junctions_f_images_[0][0][2].junction_(1) == 50.);
      REQUIRE(acm.junctions_f_images_[0][0][3].junction_(1) == 50.);
      REQUIRE(equals(acm.junctions_f_images_[0][0][4].junction_(1), 56.));
      REQUIRE(equals(acm.junctions_f_images_[0][0][5].junction_(1), 56.));

      REQUIRE(acm.arucos_corners_f_images_[0][0][3].points_f_image_(0, 2) == 51.5);
      REQUIRE(acm.arucos_corners_f_images_[0][0][3].points_f_image_(1, 2) == 54.5);
    }

    SECTION("3x3 black") {
      CharucoboardConfig ar_cfg(3, 3, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 4);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 15.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -15.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("4x3 black") {
      CharucoboardConfig ar_cfg(4, 3, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -9.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 15.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("4x4 black") {
      CharucoboardConfig ar_cfg(4, 4, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 8);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -9.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 21.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == 3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("3x3 white") {
      CharucoboardConfig ar_cfg(3, 3, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 5);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -15.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 15.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == -9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -15.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("3x4 white") {
      CharucoboardConfig ar_cfg(3, 4, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -15.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 21.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == -9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -9.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("4x3 white") {
      CharucoboardConfig ar_cfg(4, 3, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 15.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 21.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == -3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }

    SECTION("4x4 white") {
      CharucoboardConfig ar_cfg(4, 4, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 8);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == 21.);

      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](0, 2) == 21.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](1, 2) == 3.);
      REQUIRE(acm.boards_.boards_[0].arucos_corners_f_world_[3](2, 2) == -2.);
    }
  }

  TEST_CASE("Calibrate using openCV")
  {
    ModelConfig model_config{PoseGens::gen_poses_func_origin_looking_up(),
                             PoseGens::gen_poses_func_heiko_calibration_poses(),
                             camsim::CameraTypes::simulation,
                             0.1775};

    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 0.00001;
    double u_noise_sigma = 1.0;

    SECTION("CheckerboardSolverRunner") {

      CheckerboardConfig ch_cfg(12, 9, 0.030);
      CheckerboardCalibrationModel ccm(model_config, ch_cfg);

      CheckerboardSolverRunner solver_runner{ccm,
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             gtsam::Vector2::Constant(u_sampler_sigma),
                                             gtsam::Vector2::Constant(u_noise_sigma),
                                             false};


      auto result = solver_runner(solver_opencv_factory<CheckerboardCalibrationModel>());

      gtsam::equals<double> gtequal{1.e-4};
      REQUIRE(gtequal(result.calibration_.fx(), ccm.cameras_.calibration_.fx()));
      REQUIRE(gtequal(result.calibration_.fy(), ccm.cameras_.calibration_.fy()));
      REQUIRE(gtequal(result.calibration_.skew(), ccm.cameras_.calibration_.skew()));
      REQUIRE(gtequal(result.calibration_.px(), ccm.cameras_.calibration_.px()));
      REQUIRE(gtequal(result.calibration_.py(), ccm.cameras_.calibration_.py()));
      REQUIRE(gtequal(result.calibration_.k1(), ccm.cameras_.calibration_.k1()));
      REQUIRE(gtequal(result.calibration_.k2(), ccm.cameras_.calibration_.k2()));
      REQUIRE(gtequal(result.calibration_.p1(), ccm.cameras_.calibration_.p1()));
      REQUIRE(gtequal(result.calibration_.p2(), ccm.cameras_.calibration_.p2()));

    }

    SECTION("CharucoboardSolverRunner") {

      CharucoboardConfig ch_cfg(12, 9, 0.030, false, 0.0225);
      CharucoboardCalibrationModel ccm(model_config, ch_cfg);

      CharucoboardSolverRunner solver_runner{ccm,
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             gtsam::Vector2::Constant(u_sampler_sigma),
                                             gtsam::Vector2::Constant(u_noise_sigma),
                                             false};


      auto result = solver_runner(solver_opencv_factory<CharucoboardCalibrationModel>());

      gtsam::equals<double> gtequal{1.e-4};
      REQUIRE(gtequal(result.calibration_.fx(), ccm.cameras_.calibration_.fx()));
      REQUIRE(gtequal(result.calibration_.fy(), ccm.cameras_.calibration_.fy()));
      REQUIRE(gtequal(result.calibration_.skew(), ccm.cameras_.calibration_.skew()));
      REQUIRE(gtequal(result.calibration_.px(), ccm.cameras_.calibration_.px()));
      REQUIRE(gtequal(result.calibration_.py(), ccm.cameras_.calibration_.py()));
      REQUIRE(gtequal(result.calibration_.k1(), ccm.cameras_.calibration_.k1()));
      REQUIRE(gtequal(result.calibration_.k2(), ccm.cameras_.calibration_.k2()));
      REQUIRE(gtequal(result.calibration_.p1(), ccm.cameras_.calibration_.p1()));
      REQUIRE(gtequal(result.calibration_.p2(), ccm.cameras_.calibration_.p2()));
    }
  }

  TEST_CASE("Calibrate with distortion using openCV")
  {
    double r_sigma = 0.1;
    double t_sigma = 0.3;
    double u_sampler_sigma = 0.5;
    double u_noise_sigma = 1.0;

    CheckerboardConfig ch_cfg(12, 9, 0.030);

    SECTION("k1=0.1") {

      gtsam::Cal3DS2 cal3ds2(475, 475, 0, 400, 300, 0.1, 0., 0., 0.);
      ModelConfig model_config{PoseGens::gen_poses_func_origin_looking_up(),
                               PoseGens::gen_poses_func_heiko_calibration_poses(),
                               cal3ds2};

      CheckerboardCalibrationModel ccm(model_config, ch_cfg);

      CheckerboardSolverRunner solver_runner{ccm,
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             gtsam::Vector2::Constant(u_sampler_sigma),
                                             gtsam::Vector2::Constant(u_noise_sigma),
                                             false};


      auto result = solver_runner(solver_opencv_factory<CheckerboardCalibrationModel>());

      result.calibration_.print("calibration\n");

      gtsam::equals<double> gtbigequal{1.e-0};
      gtsam::equals<double> gtlitequal{1.e-2};
      REQUIRE(gtbigequal(result.calibration_.fx(), ccm.cameras_.calibration_.fx()));
      REQUIRE(gtbigequal(result.calibration_.fy(), ccm.cameras_.calibration_.fy()));
      REQUIRE(gtbigequal(result.calibration_.skew(), ccm.cameras_.calibration_.skew()));
      REQUIRE(gtbigequal(result.calibration_.px(), ccm.cameras_.calibration_.px()));
      REQUIRE(gtbigequal(result.calibration_.py(), ccm.cameras_.calibration_.py()));
      REQUIRE(gtlitequal(result.calibration_.k1(), ccm.cameras_.calibration_.k1()));
      REQUIRE(gtlitequal(result.calibration_.k2(), ccm.cameras_.calibration_.k2()));
      REQUIRE(gtlitequal(result.calibration_.p1(), ccm.cameras_.calibration_.p1()));
      REQUIRE(gtlitequal(result.calibration_.p2(), ccm.cameras_.calibration_.p2()));

    }

    SECTION("k1=-0.1, k2=0.05, p1=0.04, p2=0.02") {

      gtsam::Cal3DS2 cal3ds2(475, 475, 0, 400, 300, -0.1, 0.05, 0.04, 0.02);
      ModelConfig model_config{PoseGens::gen_poses_func_origin_looking_up(),
                               PoseGens::gen_poses_func_heiko_calibration_poses(),
                               cal3ds2};

      CheckerboardCalibrationModel ccm(model_config, ch_cfg);

      CheckerboardSolverRunner solver_runner{ccm,
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             gtsam::Vector2::Constant(u_sampler_sigma),
                                             gtsam::Vector2::Constant(u_noise_sigma),
                                             false};


      auto result = solver_runner(solver_opencv_factory<CheckerboardCalibrationModel>());

      result.calibration_.print("calibration\n");

      gtsam::equals<double> gtbigequal{5.e-1};
      gtsam::equals<double> gtlitequal{1.e-2};
      REQUIRE(gtbigequal(result.calibration_.fx(), ccm.cameras_.calibration_.fx()));
      REQUIRE(gtbigequal(result.calibration_.fy(), ccm.cameras_.calibration_.fy()));
      REQUIRE(gtbigequal(result.calibration_.skew(), ccm.cameras_.calibration_.skew()));
      REQUIRE(gtbigequal(result.calibration_.px(), ccm.cameras_.calibration_.px()));
      REQUIRE(gtbigequal(result.calibration_.py(), ccm.cameras_.calibration_.py()));
      REQUIRE(gtlitequal(result.calibration_.k1(), ccm.cameras_.calibration_.k1()));
      REQUIRE(gtlitequal(result.calibration_.k2(), ccm.cameras_.calibration_.k2()));
      REQUIRE(gtlitequal(result.calibration_.p1(), ccm.cameras_.calibration_.p1()));
      REQUIRE(gtlitequal(result.calibration_.p2(), ccm.cameras_.calibration_.p2()));
    }

    SECTION("project_calibration: k1=-0.1, k2=0.05, p1=0.04, p2=0.02") {

      gtsam::Cal3DS2 cal3ds2(475, 475, 0, 400, 300, -0.1, 0.05, 0.04, 0.02);
      ModelConfig model_config{PoseGens::gen_poses_func_origin_looking_up(),
                               PoseGens::gen_poses_func_heiko_calibration_poses(),
                               cal3ds2};

      CheckerboardCalibrationModel ccm(model_config, ch_cfg);

      CheckerboardSolverRunner solver_runner{ccm,
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                               gtsam::Vector3::Constant(t_sigma)).finished(),
                                             gtsam::Vector2::Constant(u_sampler_sigma),
                                             gtsam::Vector2::Constant(u_noise_sigma),
                                             false};


      auto result = solver_runner(solver_project_calibrate_factory<CheckerboardCalibrationModel>());

      result.calibration_.print("calibration\n");

      gtsam::equals<double> gtbigequal{8.e-1};
      gtsam::equals<double> gtlitequal{1.e-2};
      REQUIRE(gtbigequal(result.calibration_.fx(), ccm.cameras_.calibration_.fx()));
      REQUIRE(gtbigequal(result.calibration_.fy(), ccm.cameras_.calibration_.fy()));
      REQUIRE(gtbigequal(result.calibration_.skew(), ccm.cameras_.calibration_.skew()));
      REQUIRE(gtsam::assert_equal(result.calibration_.px(), ccm.cameras_.calibration_.px(), 5));
      REQUIRE(gtbigequal(result.calibration_.py(), ccm.cameras_.calibration_.py()));
      REQUIRE(gtlitequal(result.calibration_.k1(), ccm.cameras_.calibration_.k1()));
      REQUIRE(gtlitequal(result.calibration_.k2(), ccm.cameras_.calibration_.k2()));
      REQUIRE(gtlitequal(result.calibration_.p1(), ccm.cameras_.calibration_.p1()));
      REQUIRE(gtlitequal(result.calibration_.p2(), ccm.cameras_.calibration_.p2()));
    }
  }

#endif
}
