
#include "cal_solver_runner.hpp"

namespace camsim
{

// ==============================================================================
// BoardData
// ==============================================================================

  template<>
  void BoardData<CheckerboardCalibrationModel>::load_board_datas(
    SolverRunnerBase<CheckerboardCalibrationModel> sr,
    const CameraModel &camera,
    const CheckerboardModel &board,
    std::vector<BoardData<CheckerboardCalibrationModel>> &board_datas)
  {
    board_datas.emplace_back(BoardData<CheckerboardCalibrationModel>{
      board,
      sr.get_perturbed_junctions_f_image(sr.model_.junctions_f_images_[camera.index()][board.index()])});
  }
}
