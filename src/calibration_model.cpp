
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


  template<typename TConfig>
  std::size_t BoardModel<TConfig>::index() const
  {
    return gtsam::Symbol(key_).index();
  }

//  template<typename TConfig>
//  std::uint64_t BoardModel<TConfig>::default_key()
//  {
//    return board_key(0);
//  }

  template<typename TConfig>
  std::uint64_t BoardModel<TConfig>::board_key(std::size_t idx)
  {
    return gtsam::Symbol{'b', idx}.key();
  }

  template<>
  std::size_t BoardModel<CharucoboardConfig>::index() const
  {
    return gtsam::Symbol(key_).index();
  }

// ==============================================================================
// Junctions
// ==============================================================================

  static std::vector<PointFBoard> gen_junctions_f_board(
    const CheckerboardConfig &bd_cfg)
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
    const CheckerboardConfig &bd_cfg,
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
        for (JunctionId i = 0; i < boards.bd_cfg_.max_junction_id_; i += 1) {
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
    const CharucoboardConfig &bd_cfg)
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
    const CharucoboardConfig &bd_cfg,
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
    const CharucoboardsModel boards)
  {
    std::vector<std::vector<std::vector<ArucoCornersFImage>>> arucos_corners_f_images;

    for (auto &camera : cameras.cameras_) {
      std::vector<std::vector<ArucoCornersFImage>> per_camera{};
      for (auto &board : boards.boards_) {
        std::vector<ArucoCornersFImage> per_board{};
        for (ArucoId i = 0; i < boards.bd_cfg_.max_aruco_id_; i += 1) {
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

  template<typename TConfig>
  BoardModel<TConfig>::BoardModel(const TConfig &bd_cfg,
                                  std::uint64_t key,
                                  const gtsam::Pose3 &board_f_world) :
    key_{key},
    board_f_world_{board_f_world},
    junctions_f_world_{gen_junctions_f_world(bd_cfg, board_f_world)}
  {}

  CharucoboardModel::CharucoboardModel(const CharucoboardConfig &bd_cfg,
                                       std::uint64_t key,
                                       const gtsam::Pose3 &board_f_world) :
    BoardModel{bd_cfg, key, board_f_world},
    arucos_corners_f_world_{gen_arucos_corners_f_world(bd_cfg, board_f_world)}
  {}

  template<typename TConfig, typename TBoardModel>
  static std::vector<TBoardModel> gen_boards(
    const ModelConfig &cfg,
    const TConfig &bd_cfg)
  {
    std::vector<TBoardModel> boards_list{};
    auto boards_f_world = gen_boards_f_world(cfg);
    for (std::size_t i = 0; i < boards_f_world.size(); i += 1) {
      auto &board_f_world = boards_f_world[i];
      boards_list.emplace_back(TBoardModel(bd_cfg,
                                           CheckerboardModel::board_key(i),
                                           board_f_world));
    }
    return boards_list;
  }

  template<typename TConfig, typename TBoardModel>
  BoardsModel<TConfig, TBoardModel>::BoardsModel(const ModelConfig &cfg,
                                                 const TConfig &bd_cfg) :
    bd_cfg_{bd_cfg},
    boards_{gen_boards<TConfig, TBoardModel>(cfg, bd_cfg)},
    junctions_f_board_{gen_junctions_f_board(bd_cfg)}
  {}

  CharucoboardsModel::CharucoboardsModel(const ModelConfig &cfg,
                                         const CharucoboardConfig &bd_cfg) :
    BoardsModel(cfg, bd_cfg),
    arucos_corners_f_board_{gen_arucos_corners_f_board(bd_cfg)}
  {}

// ==============================================================================
// Calibration models
// ==============================================================================

  // the following unspecialized constructor will be expanded by the CharucoboardCalibrationModel
  // constructor. An alternative could be to create a specialized constructor like we do below
  // for CheckerboardCalibrationModel.
  template<typename TConfig, typename TBoardModel, typename TBoardsModel>
  CalibrationModel<TConfig, TBoardModel, TBoardsModel>::CalibrationModel(
    const ModelConfig &cfg,
    const TConfig &bd_cfg) :
    BaseModel(cfg),
    bd_cfg_{bd_cfg},
    boards_{TBoardsModel(cfg, bd_cfg_)},
    junctions_f_images_{gen_junctions_f_image(cameras_, boards_)}
  {}

  // The following specialized constructor is needed because CheckerboardCalibrationModel
  // is defined with a using statement and nothing will cause the expansion of the above
  // unspecialized constructor. The linker throws an error unless this specialized constructor
  // is explicitly declared.
  template<>
  CalibrationModel<CheckerboardConfig, CheckerboardModel, CheckerboardsModel>::CalibrationModel(
    const ModelConfig &cfg,
    const CheckerboardConfig &bd_cfg) :
    BaseModel(cfg),
    bd_cfg_{bd_cfg},
    boards_{CheckerboardsModel(cfg, bd_cfg_)},
    junctions_f_images_{gen_junctions_f_image(cameras_, boards_)}
  {}

  CharucoboardCalibrationModel::CharucoboardCalibrationModel(
    const ModelConfig &cfg,
    const CharucoboardConfig &bd_cfg) :
    CalibrationModel{cfg, bd_cfg},
    arucos_corners_f_images_{gen_arucos_corners_f_images(cameras_, boards_)}
  {}

  template<>
  void CalibrationModel<CheckerboardConfig, CheckerboardModel, CheckerboardsModel>::print_junctions_f_image()
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
