
#ifndef _CAL_RUN_HPP
#define _CAL_RUN_HPP

#include <memory>

namespace camsim
{
  int cal_run();

  int cal_solver_opencv_checkerboard();

  int cal_solver_calibrate_camera();

  int cal_solver_homography();

  template<typename TModel>
  struct SolverFactoryInterface;

  template<typename TModel>
  std::unique_ptr<SolverFactoryInterface<TModel>> solver_opencv_factory();

  template<typename TModel>
  std::unique_ptr<SolverFactoryInterface<TModel>> solver_homography_factory();

  template<typename TModel>
  std::unique_ptr<SolverFactoryInterface<TModel>> solver_project_calibrate_factory();
}
#endif //_CAL_RUN_HPP
