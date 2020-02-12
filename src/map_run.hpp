
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

  class CameraModel;

  class MarkerModel;

  std::function<void(const CameraModel &, const std::vector<std::reference_wrapper<const MarkerModel>> &)>
  solver_marker_marker_factory(SolverRunner &sr);
}

#endif //_MAP_RUN_HPP
