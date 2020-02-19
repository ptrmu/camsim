
#ifndef _MAP_RUN_HPP
#define _MAP_RUN_HPP

#include <functional>
#include <vector>

namespace camsim
{
  void map_global(double r_sigma,
                  double t_sigma,
                  double u_sampler_sigma,
                  double u_noise_sigma);

  void map_global_thread(double r_sigma,
                         double t_sigma,
                         double u_sampler_sigma,
                         double u_noise_sigma);


  class SolverRunner;

  class FrameData;

  std::function<void(const FrameData &)>
  solver_marker_marker_factory(SolverRunner &sr);

  std::function<void(const FrameData &)>
  solver_project_between_factory(SolverRunner &sr, bool initial_with_truth);

  std::function<void(const FrameData &)>
  solver_project_between_repeated_factory(SolverRunner &sr);

  std::function<void(const FrameData &)>
  solver_project_between_isam_factory(SolverRunner &sr);

  std::function<void(const FrameData &)>
  solver_project_between_opencv_factory(SolverRunner &sr);
}

#endif //_MAP_RUN_HPP
