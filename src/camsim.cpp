
#include "camsim.hpp"
#include "pfm_model.hpp"

int main()
{
//  auto ret = opencv_resection();
//  auto ret = gtsam_resection();
//  auto ret = vtk_line_plot();
//  auto ret = vtk_rand_dist();
//  auto ret = vtk_multi_plot();
  auto ret = camsim::pfm_model_test();

  return ret;
}
