
#ifndef _CALIBRATION_MODEL_HPP
#define _CALIBRATION_MODEL_HPP

#include "calibration_board_config.hpp"
#include "model.hpp"

namespace camsim
{

  struct JunctionFImage
  {
    const bool visible_;
    const std::uint64_t camera_key_;
    const std::uint64_t board_key_;
    const std::uint64_t junction_id_;

    const PointFImage junction_;

    JunctionFImage(bool visible,
                   std::uint64_t camera_key,
                   std::uint64_t board_key,
                   std::uint64_t junction_id,
                   const PointFImage &junction) :
      visible_{visible},
      camera_key_{camera_key},
      board_key_{board_key},
      junction_id_{junction_id},
      junction_{junction}
    {}

    std::size_t camera_index() const; //
    std::size_t board_index() const; //
  };

  struct ArucoCornersFImage
  {
    const bool visible_;
    const std::uint64_t camera_key_;
    const std::uint64_t board_key_;
    const std::uint64_t aruco_id_;

    const CornerPointsFImage points_f_image_;

    ArucoCornersFImage(bool visible,
                       std::uint64_t camera_key,
                       std::uint64_t board_key,
                       std::uint64_t aruco_id,
                       const CornerPointsFImage &points_f_image) :
      visible_{visible},
      camera_key_{camera_key},
      board_key_{board_key},
      aruco_id_{aruco_id},
      points_f_image_{points_f_image}
    {}

    std::size_t camera_index() const; //
    std::size_t board_index() const; //
  };

  struct CalibrationResult
  {
    const gtsam::Cal3DS2 calibration_;

    CalibrationResult(const gtsam::Cal3DS2 &calibration) :
      calibration_{calibration}
    {}
  };

// ==============================================================================
// Board models
// ==============================================================================

  template<typename TConfig>
  struct BoardModel
  {
    const std::uint64_t key_;
    const gtsam::Pose3 board_f_world_;
    const std::vector<PointFWorld> junctions_f_world_;

    BoardModel(const TConfig &bd_cfg,
               std::uint64_t key,
               const gtsam::Pose3 &board_f_world);

    std::size_t index() const; //
    static std::uint64_t default_key(); //
    static std::uint64_t board_key(std::size_t idx); //
  };

  using CheckerboardModel = BoardModel<CheckerboardConfig>;

  struct CharucoboardModel : public BoardModel<CharucoboardConfig>
  {
    const std::vector<CornerPointsFWorld> arucos_corners_f_world_;

    CharucoboardModel(const CharucoboardConfig &bd_cfg,
                      std::uint64_t key,
                      const gtsam::Pose3 &board_f_world);
  };

// ==============================================================================
// Boards models
// ==============================================================================

  template<typename TConfig, typename TBoardModel>
  struct BoardsModel
  {
    const TConfig &bd_cfg_;
    const std::vector<TBoardModel> boards_; // [board]
    const std::vector<PointFBoard> junctions_f_board_; // [junction]

    BoardsModel(const ModelConfig &cfg,
                const TConfig &bd_cfg);
  };

  using CheckerboardsModel = BoardsModel<CheckerboardConfig, CheckerboardModel>;

  struct CharucoboardsModel : public BoardsModel<CharucoboardConfig, CharucoboardModel>
  {
    const std::vector<CornerPointsFBoard> arucos_corners_f_board_; // [marker]

    CharucoboardsModel(const ModelConfig &cfg,
                       const CharucoboardConfig &bd_cfg);
  };

// ==============================================================================
// Calibration models
// ==============================================================================

  template<typename TConfig, typename TBoardModel, typename TBoardsModel>
  struct CalibrationModel : public BaseModel
  {
    using Config = TConfig; //
    using TargetModel = TBoardModel; //
    using TargetsModel = TBoardsModel; //
    using Result = CalibrationResult; //

    const Config bd_cfg_;
    const TargetsModel boards_;
    const std::vector<std::vector<std::vector<JunctionFImage>>> junctions_f_images_; // [camera][board][junction]

    explicit CalibrationModel(const ModelConfig &cfg,
                              const Config &bd_cfg);

    void print_junctions_f_image();
  };

  using CheckerboardCalibrationModel = CalibrationModel<CheckerboardConfig, CheckerboardModel, CheckerboardsModel>;

  struct CharucoboardCalibrationModel : CalibrationModel<CharucoboardConfig, CharucoboardModel, CharucoboardsModel>
  {
    const std::vector<std::vector<std::vector<ArucoCornersFImage>>> arucos_corners_f_images_; // [camera][board][aruco]

    CharucoboardCalibrationModel(const ModelConfig &cfg,
                                 const CharucoboardConfig &bd_cfg);
  };

}
#endif //_CALIBRATION_MODEL_HPP
