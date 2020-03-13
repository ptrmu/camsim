
#include "calibration_model.hpp"

#include "gtsam/inference/Symbol.h"

namespace camsim
{

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

  static std::vector<PointFWorld> gen_junctions_f_world(
    const CheckerboardCalibrationTypes::Config &bd_cfg,
    const gtsam::Pose3 &board_f_world)
  {
    std::vector<PointFWorld> junctions_f_world{};
    for (JunctionId i = 0; i < bd_cfg.max_junction_id_; i += 1) {
      auto junction_f_facade = bd_cfg.to_junction_location(i);
      auto junction_f_board = bd_cfg.to_point_f_board(junction_f_facade);
      junctions_f_world.emplace_back(PointFWorld(board_f_world * junction_f_board));
    }
    return junctions_f_world;
  }

  static std::vector<ArucoCornersFWorld> gen_aruco_corners_f_world(
    const CharucoboardCalibrationTypes::Config &bd_cfg,
    const gtsam::Pose3 &board_f_world)
  {
    std::vector<ArucoCornersFWorld> aruco_corners_f_world{};
//    for (JunctionId i = 0; i < bd_cfg.max_junction_id_; i += 1) {
//      auto junction_f_facade = bd_cfg.to_junction_location(i);
//      auto junction_f_board = bd_cfg.to_point_f_board(junction_f_facade);
//      aruco_corners_f_world.emplace_back(PointFWorld(board_f_world * junction_f_board));
//    }
    return aruco_corners_f_world;
  }

  static std::vector<PointFBoard> gen_junctions_f_board(
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    return std::vector<PointFBoard>{};
  }

  static std::vector<ArucoCornersFBoard> gen_aruco_corners_f_board(
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {
    return std::vector<ArucoCornersFBoard>{};
  }

  static std::vector<std::vector<std::vector<JunctionFImage>>> gen_junctions_f_images(
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    return std::vector<std::vector<std::vector<JunctionFImage>>>{};
  }

  static std::vector<std::vector<std::vector<ArucoCornersFImage>>> gen_aruco_corners_f_images(
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {
    return std::vector<std::vector<std::vector<ArucoCornersFImage>>>{};
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
                             gen_aruco_corners_f_world(bd_cfg, board_f_world)};

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
                                                            gen_aruco_corners_f_board(bd_cfg));
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
    junctions_f_images_{gen_junctions_f_images(bd_cfg)}
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
    aruco_corners_f_images_{gen_aruco_corners_f_images(bd_cfg)}
  {}

}
