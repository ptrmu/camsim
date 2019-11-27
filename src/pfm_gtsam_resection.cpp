

#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "pfm_run.hpp"
#include "pfm_model.hpp"

using gtsam::symbol_shorthand::X;

namespace camsim
{
/**
  * Unary factor on the unknown pose, resulting from measuring the projection of
    * a known 3D point in the image
  */
  class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    typedef NoiseModelFactor1 <gtsam::Pose3> Base;

    const gtsam::Cal3_S2 &K_;     ///< camera's intrinsic parameters
    const gtsam::Point3 P_;       ///< 3D point on the calibration rig
    const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

  public:

    /// Construct factor given known point P and its projection p
    ResectioningFactor(const gtsam::SharedNoiseModel &model, const gtsam::Key &key,
                       const gtsam::Cal3_S2 &calib, const gtsam::Point2 &p, const gtsam::Point3 &P) :
      Base(model, key), K_(calib), P_(P), p_(p)
    {
    }

    /// evaluate the error
    virtual gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                        boost::optional<gtsam::Matrix &> H = boost::none) const
    {
      gtsam::SimpleCamera camera(pose, K_);
      return camera.project(P_, H, boost::none, boost::none) - p_;
    }
  };


  void pfm_gtsam_resection(const gtsam::Cal3_S2 &camera_calibration,
                           const std::vector<gtsam::Point2> &corners_f_image,
                           const std::vector<gtsam::Point3> &corners_f_world,
                           const gtsam::Pose3 &camera_f_world_initial,
                           const gtsam::SharedNoiseModel &measurement_noise,
                           gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance)
  {
    /* 1. create graph */
    gtsam::NonlinearFactorGraph graph;

    /* 2. add measurement factors to the graph */
    for (int i = 0; i < corners_f_world.size(); i += 1) {
      graph.emplace_shared<ResectioningFactor>(measurement_noise, X(1),
                                               camera_calibration,
                                               corners_f_image[i],
                                               corners_f_world[i]);
    }

    /* 3. Create an initial estimate for the camera pose */
    gtsam::Values initial;
    initial.insert(X(1), camera_f_world_initial);

    /* 4. Optimize the graph using Levenberg-Marquardt*/
    auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    camera_f_world = result.at<gtsam::Pose3>(X(1));

    gtsam::Marginals marginals(graph, result);
    camera_f_world_covariance = marginals.marginalCovariance(X(1));
  }
}
