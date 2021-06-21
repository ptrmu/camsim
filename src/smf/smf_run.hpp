
#pragma once

int sfm_example_smart_factor(void); //
int isam2_example_smart_factor(void); //
int fixed_lag_smoother_example(void); //
int concurrent_filtering_and_smoothing_example(void); //

namespace camsim
{
  int smart_factor_pose_simple(void); //
  int imager_relative_pose(void); //
  int factors_gtsam_test(void); //
  int inter_marker_pose_test(void); //
}
