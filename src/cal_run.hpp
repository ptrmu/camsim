
#ifndef _CAL_RUN_HPP
#define _CAL_RUN_HPP

#include <memory>

namespace camsim
{
  int cal_run();

  int cal_solver_opencv_checkerboard();

  int cal_solver();

  template<typename TCalibrationModel>
  struct SolverFactoryInterface;

  template<typename TCalibrationModel>
  std::unique_ptr<SolverFactoryInterface<TCalibrationModel>> solver_opencv_factory();
}
#endif //_CAL_RUN_HPP
