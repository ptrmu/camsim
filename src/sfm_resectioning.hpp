
#ifndef _SFM_RESECTION_HPP
#define _SFM_RESECTION_HPP

#include <tuple>
#include <gtsam/linear/NoiseModel.h>

namespace gtsam
{
  class Cal3_S2;
  class Pose3;
  class Point2;
}

namespace camsim
{
  class SfmModel;

  std::vector<std::tuple<gtsam::Pose3, gtsam::Matrix6>> sfm_resectioning(
    const SfmModel &sfm_model,
    const gtsam::Cal3_S2 &K,
    const gtsam::SharedNoiseModel &measurement_noise,
    const std::vector<std::vector<gtsam::Point2>> &corners_f_images);
}
#endif //_SFM_RESECTION_HPP
