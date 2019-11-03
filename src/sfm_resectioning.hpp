
#ifndef _SFM_RESECTION_HPP
#define _SFM_RESECTION_HPP

#include <tuple>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/geometry/Point3.h>

namespace gtsam
{
  class Cal3_S2;

  class Pose3;

  class Point2;
}

namespace camsim
{
  class CalcCameraPoseImpl;

  class CalcCameraPose
  {
    std::unique_ptr<CalcCameraPoseImpl> impl_;

  public:
    CalcCameraPose(const gtsam::Cal3_S2 &K,
                   const gtsam::SharedNoiseModel &measurement_noise,
                   const std::vector<gtsam::Point3> &corners_f_marker);

    ~CalcCameraPose();

    std::tuple<gtsam::Pose3, gtsam::Matrix6> camera_f_marker(
      const std::vector<gtsam::Point2> &corners_f_image);
  };
}

int sfm_run_resectioning();

#endif //_SFM_RESECTION_HPP
