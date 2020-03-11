
#ifndef _CALIBRATION_MODEL_HPP
#define _CALIBRATION_MODEL_HPP

#include "calibration_board_config.hpp"
#include "model.hpp"

namespace camsim
{
  using PointFWorld = gtsam::Point3;
  using PointFBoard = gtsam::Point3;
  using PointFImage = gtsam::Point2;
  using CornerPointsFWorld = std::array<PointFWorld, 4>;
  using CornerPointsFBoard = std::array<PointFBoard, 4>;
  using CornerPointsFImage = std::array<PointFImage, 4>;

  struct JunctionFImage
  {
    const bool visible_;
    const gtsam::Point2 junction_;

    JunctionFImage(bool visible, const gtsam::Point2 &junction) :
      visible_{visible}, junction_{junction}
    {}
  };


  struct ArucoCornersFWorld
  {
    const CornerPointsFWorld corners_f_world_;

    ArucoCornersFWorld(const CornerPointsFWorld &corners_f_world) :
      corners_f_world_{corners_f_world}
    {}
  };

  struct ArucoCornersFBoard
  {
    const CornerPointsFBoard corners_t_board_;

    ArucoCornersFBoard(const CornerPointsFBoard &corners_f_board) :
      corners_t_board_{corners_f_board}
    {}
  };

  struct ArucoCornersFImage
  {
    const bool visible_;
    const CornerPointsFImage points_f_image_;

    ArucoCornersFImage(bool visible, const CornerPointsFImage &points_f_image) :
      visible_{visible}, points_f_image_{points_f_image}
    {}
  };

// ==============================================================================
// Types structure
// ==============================================================================

  template<class TTypes>
  class CheckerboardsModel;

  template<class TTypes>
  class CharucoboardsModel;

  struct CheckerboardCalibrationTypes
  {
    using Config = class CheckerboardConfig;
    using BoardModel = class CheckerboardModel;
    template<class TTypes>
    using BoardsModel = class CheckerboardsModel<TTypes>;
  };

  struct CharucoboardCalibrationTypes
  {
    using Config = class CharucoboardConfig;
    using BoardModel = class CharucoboardModel;
    template<class TTypes>
    using BoardsModel = struct CharucoboardsModel<TTypes>;
  };

// ==============================================================================
// Init routines
// ==============================================================================

  struct Init
  {
    template<class TTypes>
    static const typename TTypes::template BoardsModel<TTypes> gen_boards(
      const ModelConfig &cfg,
      const typename TTypes::Config &bd_cfg);

    template<>
    static CheckerboardCalibrationTypes::BoardModel gen_boards<CheckerboardCalibrationTypes>(
      const ModelConfig &cfg,
      const CheckerboardCalibrationTypes::Config &bd_cfg);

    template<class TTypes>
    static std::vector<std::vector<std::vector<JunctionFImage>>> gen_junctions_f_images(
      const typename TTypes::Config &bd_cfg)
    {
      return std::vector<std::vector<std::vector<JunctionFImage>>>{};
    }

    template<class TTypes>
    static std::vector<std::vector<std::vector<ArucoCornersFImage>>> gen_aruco_corners_f_images(
      const typename TTypes::Config &bd_cfg)
    {
      return std::vector<std::vector<std::vector<ArucoCornersFImage>>>{};
    }
  };

// ==============================================================================
// Board models
// ==============================================================================

  struct CheckerboardModel
  {
    const std::uint64_t key_;
    const gtsam::Pose3 board_f_world_;
    const std::vector<PointFWorld> junctions_f_world_;

    CheckerboardModel(std::uint64_t key,
                      const gtsam::Pose3 &board_f_world,
                      const std::vector<PointFWorld> &junctions_f_world) :
      key_{key}, board_f_world_{board_f_world}, junctions_f_world_{junctions_f_world}
    {}
  };

  struct CharucoboardModel : public CheckerboardModel
  {
    const std::vector<ArucoCornersFWorld> aruco_corners_f_world_;

    CharucoboardModel(std::uint64_t key,
                      const gtsam::Pose3 &board_f_world,
                      const std::vector<PointFWorld> &junctions_f_world,
                      const std::vector<ArucoCornersFWorld> &aruco_corners_f_world) :
      CheckerboardModel{key, board_f_world, junctions_f_world},
      aruco_corners_f_world_{aruco_corners_f_world}
    {}
  };

// ==============================================================================
// Boards models
// ==============================================================================

  template<class TTypes>
  struct CheckerboardsModel
  {
    const typename TTypes::Config &ch_cfg_;
    const std::vector<typename TTypes::BoardModel> boards_; // [board]
    const std::vector<PointFBoard> junctions_f_board_; // [junction]

    CheckerboardsModel(const ModelConfig &cfg,
                       const typename TTypes::Config &ch_cfg,
                       const std::vector<typename TTypes::BoardModel> &boards,
                       const std::vector<PointFBoard> &junctions_f_board) :
      ch_cfg_{ch_cfg},
      boards_{boards},
      junctions_f_board_{junctions_f_board}
    {}
  };

  template<class TTypes>
  struct CharucoboardsModel : public CheckerboardsModel<TTypes>
  {
    const typename TTypes::Config &ar_cfg_;
    const std::vector<ArucoCornersFBoard> aruco_corners_f_board_; // [marker]

    CharucoboardsModel(const ModelConfig &cfg,
                       const typename TTypes::Config &ar_cfg,
                       const std::vector<typename TTypes::BoardModel> &boards,
                       const std::vector<PointFBoard> &junctions_f_board,
                       const std::vector<ArucoCornersFBoard> &aruco_corners_f_board) :
      CheckerboardsModel<TTypes>{ar_cfg, boards, junctions_f_board},
      ar_cfg_{ar_cfg}, aruco_corners_f_board_{aruco_corners_f_board}
    {}
  };

// ==============================================================================
// Calibration models
// ==============================================================================

  template<class TTypes>
  struct CalibrationModel : public BaseModel
  {
    const typename TTypes::Config bd_cfg_;
    const typename TTypes::template BoardsModel<TTypes> boards_;
    const std::vector<std::vector<std::vector<JunctionFImage>>> junctions_f_images_; // [camera][board][junction]

    explicit CalibrationModel(const ModelConfig &cfg, const typename TTypes::Config &bd_cfg) :
      BaseModel{cfg},
      bd_cfg_{bd_cfg},
      boards_{Init::gen_boards<TTypes>(cfg, bd_cfg_)},
      junctions_f_images_{Init::gen_junctions_f_images<TTypes>(bd_cfg)}
    {}
  };

  struct CheckerboardCalibrationModel : CalibrationModel<CheckerboardCalibrationTypes>
  {
    CheckerboardCalibrationModel(const ModelConfig &cfg,
                                 const typename CheckerboardCalibrationTypes::Config &bd_cfg) :
      CalibrationModel<CheckerboardCalibrationTypes>{cfg, bd_cfg}
    {}
  };

  struct CharucoboardCalibrationModel : CalibrationModel<CharucoboardCalibrationTypes>
  {
    const std::vector<std::vector<std::vector<ArucoCornersFImage>>> aruco_corners_f_images_; // [camera][board][aruco]

    CharucoboardCalibrationModel(const ModelConfig &cfg,
                                 const typename CharucoboardCalibrationTypes::Config &bd_cfg) :
      CalibrationModel<CharucoboardCalibrationTypes>{cfg, bd_cfg},
      aruco_corners_f_images_{Init::gen_aruco_corners_f_images<CharucoboardCalibrationTypes>(bd_cfg)}
    {}
  };

}
#endif //_CALIBRATION_MODEL_HPP
