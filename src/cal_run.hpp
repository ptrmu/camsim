
#ifndef _CAL_RUN_HPP
#define _CAL_RUN_HPP

#include <functional>

namespace camsim
{
  int cal_run();

  int cal_solver_opencv_checkerboard();

  int cal_solver();

  template<typename TCalibrationModel>
  struct FrameData;
  template<typename TCalibrationModel>
  struct SolverRunner;

  template<typename TCalibrationModel>
  const typename SolverRunner<TCalibrationModel>::SolverFactoryFunc
  solver_opencv_factory(SolverRunner<TCalibrationModel> &sr);
}
#endif //_CAL_RUN_HPP
