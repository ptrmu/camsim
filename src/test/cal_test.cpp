
#include "catch2/catch.hpp"
#include "../cal_run.hpp"
#include "../calibration_model.hpp"
#include "../model.hpp"

namespace camsim
{
#if 1

  TEST_CASE("aruco locations")
  {
    SECTION("3x3 black") {
      CharucoboardConfig ar_cfg(3, 3, 12, false, 6);
      REQUIRE(ar_cfg.to_aruco_location(0).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(1).x() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(2).x() == 30.);
      REQUIRE(ar_cfg.to_aruco_location(3).x() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(0).y() == 6.);
      REQUIRE(ar_cfg.to_aruco_location(1).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(2).y() == 18.);
      REQUIRE(ar_cfg.to_aruco_location(3).y() == 30.);

//      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 0) == -3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 1) == 3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 2) == 3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(0, 3) == -3.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 0) == -15.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 1) == -15.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 2) == -9.);
      REQUIRE(ar_cfg.to_aruco_corners_f_board(ar_cfg.to_aruco_corners_f_facade(0))(1, 3) == -9.);
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
    }
  }

  TEST_CASE("Charucoboard Model generic generation")
  {
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
                             camsim::CameraTypes::simulation,
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
      REQUIRE(acm.boards_.junctions_f_board_[0].y() == 1 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[1].y() == 1 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[2].y() == 2 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[3].y() == 2 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[4].y() == 3 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.junctions_f_board_[5].y() == 3 * square_length - squares_y * square_length / 2);
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
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[0].y() == 1 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[1].y() == 1 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[2].y() == 2 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[3].y() == 2 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[4].y() == 3 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[5].y() == 3 * square_length - squares_y * square_length / 2);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[0].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[1].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[2].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[3].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[4].z() == -2.0);
      REQUIRE(acm.boards_.boards_[0].junctions_f_world_[5].z() == -2.0);

      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](1, 0) == 3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](0, 3) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[3](1, 3) == 9.);
    }

    SECTION("3x3 black") {
      CharucoboardConfig ar_cfg(3, 3, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 4);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -3.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -15.);
    }

    SECTION("4x3 black") {
      CharucoboardConfig ar_cfg(4, 3, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -9.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -15.);
    }

    SECTION("4x4 black") {
      CharucoboardConfig ar_cfg(4, 4, 12.0, false, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 8);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -9.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -21.);
    }

    SECTION("3x3 white") {
      CharucoboardConfig ar_cfg(3, 3, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 5);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -15.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -15.);
    }

    SECTION("3x4 white") {
      CharucoboardConfig ar_cfg(3, 4, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -15.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -21.);
    }

    SECTION("4x3 white") {
      CharucoboardConfig ar_cfg(4, 3, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 6);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -15.);
    }

    SECTION("4x4 white") {
      CharucoboardConfig ar_cfg(4, 4, 12.0, true, 6.0);
      CharucoboardCalibrationModel acm(model_config, ar_cfg);
      REQUIRE(acm.boards_.arucos_corners_f_board_.size() == 8);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](0, 0) == -21.);
      REQUIRE(acm.boards_.arucos_corners_f_board_[0](1, 0) == -21.);
    }
  }

#endif
}
