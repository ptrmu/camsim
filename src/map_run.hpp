
#ifndef _MAP_RUN_HPP
#define _MAP_RUN_HPP

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
}

#endif //_MAP_RUN_HPP
