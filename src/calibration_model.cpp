
#include "calibration_model.hpp"

#include <gtsam/geometry/CalibratedCamera.h>
#include "gtsam/inference/Symbol.h"
#include "pose_with_covariance.hpp"

namespace camsim
{


  std::size_t JunctionFImage::camera_index() const
  {
    return gtsam::Symbol(camera_key_).index();
  }

  std::size_t JunctionFImage::board_index() const
  {
    return gtsam::Symbol(board_key_).index();
  }

  std::size_t ArucoCornersFImage::camera_index() const
  {
    return gtsam::Symbol(camera_key_).index();
  }

  std::size_t ArucoCornersFImage::board_index() const
  {
    return gtsam::Symbol(board_key_).index();
  }


  std::size_t CheckerboardModel::index() const
  {
    return gtsam::Symbol(key_).index();
  }

  std::uint64_t CheckerboardModel::default_key()
  {
    return board_key(0);
  }

  std::uint64_t CheckerboardModel::board_key(std::size_t idx)
  {
    return gtsam::Symbol{'b', idx}.key();
  }

// ==============================================================================
// Junctions
// ==============================================================================

  static std::vector<PointFBoard> gen_junctions_f_board(
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    std::vector<PointFBoard> junctions_f_board{};
    for (JunctionId i = 0; i < bd_cfg.max_junction_id_; i += 1) {
      auto junction_f_facade = bd_cfg.to_junction_location(i);
      auto junction_f_board = bd_cfg.to_point_f_board(junction_f_facade);
      junctions_f_board.emplace_back(junction_f_board);
    }
    return junctions_f_board;
  }

  static std::vector<PointFWorld> gen_junctions_f_world(
    const CheckerboardCalibrationTypes::Config &bd_cfg,
    const gtsam::Pose3 &board_f_world)
  {
    std::vector<PointFWorld> junctions_f_world{};
    auto junctions_f_board{gen_junctions_f_board(bd_cfg)};
    for (auto &junction_f_board : junctions_f_board) {
      junctions_f_world.emplace_back(PointFWorld(board_f_world * junction_f_board));
    }
    return junctions_f_world;
  }

  template<typename TBoardsModel>
  static std::vector<std::vector<std::vector<JunctionFImage>>> gen_junctions_f_image(
    const CamerasModel &cameras,
    const TBoardsModel &boards)
  {
    std::vector<std::vector<std::vector<JunctionFImage>>> junctions_f_image;

    for (auto &camera : cameras.cameras_) {
      std::vector<std::vector<JunctionFImage>> per_camera{};
      for (auto &board : boards.boards_) {
        std::vector<JunctionFImage> per_board{};
        for (JunctionId i = 0; i < boards.ch_cfg_.max_junction_id_; i += 1) {
          auto junction_f_world(board.junctions_f_world_[i]);
          bool visible = false;
          auto point_f_image{PointFImage{}.setZero()};

          try {
            // project the junction onto the camera's image plane.
            point_f_image = cameras.project_func_(camera.camera_f_world_,
                                                  junction_f_world,
                                                  boost::none);

            // If the point is outside of the image boundary, then don't save any of the points. This
            // simulates when a marker can't be seen by a camera.
            if (point_f_image.x() >= 0 && point_f_image.x() < 2 * cameras.calibration_.px() &&
                point_f_image.y() >= 0 && point_f_image.y() < 2 * cameras.calibration_.py()) {
              visible = true;
            }
          }
            // If the point can't be projected, then don't save any of the points. This
            // simulates when a marker can't be seen by a camera.
          catch (gtsam::CheiralityException &e) {
          }

          // Add a junction to the junction list.
          per_board.emplace_back(JunctionFImage{visible, camera.key_, board.key_, i, point_f_image});
        }

        // Add the per_board list
        per_camera.emplace_back(per_board);
      }

      // add the per_camera list
      junctions_f_image.emplace_back(per_camera);
    }

    return junctions_f_image;
  }

// ==============================================================================
// Aruco Corners
// ==============================================================================

  static std::vector<CornerPointsFBoard> gen_arucos_corners_f_board(
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {
    std::vector<CornerPointsFBoard> arucos_corners_f_board{};
    for (ArucoId i = 0; i < bd_cfg.max_aruco_id_; i += 1) {
      auto aruco_f_facade = bd_cfg.to_aruco_corners_f_facade(i);
      auto aruco_corners_f_board = bd_cfg.to_aruco_corners_f_board(aruco_f_facade);
      arucos_corners_f_board.emplace_back(aruco_corners_f_board);
    }
    return arucos_corners_f_board;
  }

  static std::vector<CornerPointsFWorld> gen_arucos_corners_f_world(
    const CharucoboardCalibrationTypes::Config &bd_cfg,
    const gtsam::Pose3 &board_f_world)
  {
    std::vector<CornerPointsFWorld> arucos_corners_f_world{};
    auto arucos_corners_f_board{gen_arucos_corners_f_board(bd_cfg)};
    for (auto &aruco_corners_f_board : arucos_corners_f_board) {
      arucos_corners_f_world.emplace_back((CornerPointsFWorld{}
        << board_f_world * aruco_corners_f_board.col(0),
        board_f_world * aruco_corners_f_board.col(1),
        board_f_world * aruco_corners_f_board.col(2),
        board_f_world * aruco_corners_f_board.col(3)).finished());
    }
    return arucos_corners_f_world;
  }

  static std::vector<std::vector<std::vector<ArucoCornersFImage>>> gen_arucos_corners_f_images(
    const CamerasModel &cameras,
    const CharucoboardsModel<CharucoboardCalibrationTypes> boards)
  {
    std::vector<std::vector<std::vector<ArucoCornersFImage>>> arucos_corners_f_images;

    for (auto &camera : cameras.cameras_) {
      std::vector<std::vector<ArucoCornersFImage>> per_camera{};
      for (auto &board : boards.boards_) {
        std::vector<ArucoCornersFImage> per_board{};
        for (ArucoId i = 0; i < boards.ar_cfg_.max_aruco_id_; i += 1) {
          auto &aruco_corners_f_world(board.arucos_corners_f_world_[i]);
          bool visible = false;
          auto corner_points_f_image{CornerPointsFImage{}.setZero()};

          try {
            for (int i = 0; i < 4; i += 1) {
              // Project the junction onto the camera's image plane for each of the corner points
              corner_points_f_image.col(i) = cameras.project_func_(camera.camera_f_world_,
                                                                   aruco_corners_f_world.col(i),
                                                                   boost::none);

              // If the point is outside of the image boundary, then don't save any of the points. This
              // simulates when a marker can't be seen by a camera.
              if (corner_points_f_image(0, i) >= 0 && corner_points_f_image(0, i) < 2 * cameras.calibration_.px() &&
                  corner_points_f_image(1, i) >= 0 && corner_points_f_image(1, i) < 2 * cameras.calibration_.py() &&
                  i == 3) {
                visible = true;
              }
            }
          }
            // If the point can't be projected, then don't save any of the points. This
            // simulates when a marker can't be seen by a camera.
          catch (gtsam::CheiralityException &e) {
          }

          // Add a junction to the junction list.
          per_board.emplace_back(ArucoCornersFImage{visible, camera.key_, board.key_, i, corner_points_f_image});
        }

        // Add the per_board list
        per_camera.emplace_back(per_board);
      }

      // add the per_camera list
      arucos_corners_f_images.emplace_back(per_camera);
    }

    return arucos_corners_f_images;
  }

// ==============================================================================
// Boards and Board models
// ==============================================================================

  static std::vector<gtsam::Pose3> gen_boards_f_world(
    const ModelConfig &cfg)
  {
    return (cfg.markers_configuration_ == MarkersConfigurations::generator) ?
           cfg.marker_pose_generator_() : std::vector<gtsam::Pose3>{};
  }

  static CheckerboardModel gen_board_model(const CheckerboardConfig &bd_cfg,
                                           std::uint64_t key,
                                           const gtsam::Pose3 &board_f_world)
  {
    return CheckerboardModel{key,
                             board_f_world,
                             gen_junctions_f_world(bd_cfg, board_f_world)};
  }

  static CharucoboardModel gen_board_model(const CharucoboardConfig &bd_cfg,
                                           std::uint64_t key,
                                           const gtsam::Pose3 &board_f_world)
  {
    return CharucoboardModel{key,
                             board_f_world,
                             gen_junctions_f_world(bd_cfg, board_f_world),
                             gen_arucos_corners_f_world(bd_cfg, board_f_world)};

  }

  template<typename TTypes>
  static std::vector<typename TTypes::BoardModel> gen_boards_list(
    const ModelConfig &cfg,
    const typename TTypes::Config &bd_cfg)
  {
    std::vector<typename TTypes::BoardModel> boards_list{};
    auto boards_f_world = gen_boards_f_world(cfg);
    for (std::size_t i = 0; i < boards_f_world.size(); i += 1) {
      auto &board_f_world = boards_f_world[i];
      boards_list.emplace_back(gen_board_model(bd_cfg,
                                               CheckerboardModel::board_key(i),
                                               board_f_world));
    }
    return boards_list;
  }

  static CheckerboardsModel<CheckerboardCalibrationTypes> gen_boards_model(
    const ModelConfig &cfg,
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    return CheckerboardsModel<CheckerboardCalibrationTypes>(cfg, bd_cfg,
                                                            gen_boards_list<CheckerboardCalibrationTypes>(cfg, bd_cfg),
                                                            gen_junctions_f_board(bd_cfg));
  }

  static CharucoboardsModel<CharucoboardCalibrationTypes> gen_boards_model(
    const ModelConfig &cfg,
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {
    return CharucoboardsModel<CharucoboardCalibrationTypes>(cfg, bd_cfg,
                                                            gen_boards_list<CharucoboardCalibrationTypes>(cfg, bd_cfg),
                                                            gen_junctions_f_board(bd_cfg),
                                                            gen_arucos_corners_f_board(bd_cfg));
  }

// ==============================================================================
// Calibration models
// ==============================================================================

  template<typename TTypes>
  CalibrationModel<TTypes>::CalibrationModel(
    const ModelConfig &cfg,
    const typename TTypes::Config &bd_cfg) :
    BaseModel{cfg},
    bd_cfg_{bd_cfg},
    boards_{gen_boards_model(cfg, bd_cfg_)},
    junctions_f_images_{gen_junctions_f_image<typename TTypes::template BoardsModel<TTypes>>(cameras_, boards_)}
  {}

  CheckerboardCalibrationModel::CheckerboardCalibrationModel(
    const ModelConfig &cfg,
    const typename CheckerboardCalibrationTypes::Config &bd_cfg) :
    CalibrationModel<CheckerboardCalibrationTypes>{cfg, bd_cfg}
  {}

  CharucoboardCalibrationModel::CharucoboardCalibrationModel(
    const ModelConfig &cfg,
    const typename CharucoboardCalibrationTypes::Config &bd_cfg) :
    CalibrationModel<CharucoboardCalibrationTypes>{cfg, bd_cfg},
    arucos_corners_f_images_{gen_arucos_corners_f_images(cameras_, boards_)}
  {}


  void CheckerboardCalibrationModel::print_junctions_f_image()
  {
    std::cout << "junctions_f_images" << std::endl;
    for (auto &per_camera : junctions_f_images_) {
      for (auto &per_board : per_camera) {
        for (auto &per_junction : per_board) {
          if (per_junction.junction_id_ == 0) {
            auto &camera = cameras_.cameras_[per_junction.camera_index()];
            auto &board = boards_.boards_[per_junction.board_index()];
            std::cout << "camera" << camera.index() << PoseWithCovariance::to_str(camera.camera_f_world_)
                      << " board" << board.index() << PoseWithCovariance::to_str(board.board_f_world_)
                      << std::endl;
          }

          auto &junction_f_image(per_junction.junction_);

          // If the junction is not visible, then it was not visible to the camera.
          if (!per_junction.visible_) {
            std::cout << "  not visible";
          } else {
            std::cout << "  " << junction_f_image.transpose();
          }
          if (per_junction.junction_id_ % (bd_cfg_.squares_x_ - 1) == (bd_cfg_.squares_x_ - 2)) {
            std::cout << std::endl;
          }
        }
        std::cout << std::endl;
      }
    }
    std::cout << std::endl;
  }

}
