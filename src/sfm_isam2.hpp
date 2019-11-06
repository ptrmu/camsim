
#ifndef _SFM_ISAM2_HPP
#define _SFM_ISAM2_HPP

#include "sfm_resectioning.hpp"

namespace camsim
{
  class SfmIsam2Impl;

  class SfmIsam2
  {
    std::unique_ptr<SfmIsam2Impl> impl_;

  public:
    SfmIsam2();

    ~SfmIsam2();

    void add_measurements(int camera_id, const std::vector<SfmPoseWithCovariance> &camera_f_markers);
  };
}

int sfm_run_isam2();

#endif //_SFM_ISAM2_HPP
