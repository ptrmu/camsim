
#include "calibration_model.hpp"

namespace camsim
{
  static std::vector<CheckerboardCalibrationTypes::BoardModel> gen_boards_list(
    const ModelConfig &cfg,
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    std::vector<CheckerboardCalibrationTypes::BoardModel> boards_list{};

  }

  static std::vector<CharucoboardCalibrationTypes::BoardModel> gen_boards_list(
    const ModelConfig &cfg,
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {

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

  const CheckerboardsModel<CheckerboardCalibrationTypes> CalibrationModelInit::gen_boards_model(
    const ModelConfig &cfg,
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {
    return CheckerboardsModel<CheckerboardCalibrationTypes>(cfg, bd_cfg,
                                                            gen_boards_list(cfg, bd_cfg),
                                                            gen_junctions_f_board(bd_cfg));
  }

  const CharucoboardsModel<CharucoboardCalibrationTypes> CalibrationModelInit::gen_boards_model(
    const ModelConfig &cfg,
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {
    return CharucoboardsModel<CharucoboardCalibrationTypes>(cfg, bd_cfg,
                                                            gen_boards_list(cfg, bd_cfg),
                                                            gen_junctions_f_board(bd_cfg),
                                                            gen_aruco_corners_f_board(bd_cfg));
  }

  const std::vector<std::vector<std::vector<JunctionFImage>>> CalibrationModelInit::gen_junctions_f_images(
    const CheckerboardCalibrationTypes::Config &bd_cfg)
  {}

  const std::vector<std::vector<std::vector<JunctionFImage>>> CalibrationModelInit::gen_junctions_f_images(
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {}


  const std::vector<std::vector<std::vector<ArucoCornersFImage>>> CalibrationModelInit::gen_aruco_corners_f_images(
    const CharucoboardCalibrationTypes::Config &bd_cfg)
  {}
}
