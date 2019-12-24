
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "model.hpp"
#include "pfp_run.hpp"

using gtsam::symbol_shorthand::X;

namespace camsim
{
  using namespace gtsam;

  class TransformFromFactor : public NoiseModelFactor1<Pose3>
  {
    typedef NoiseModelFactor1<Pose3> Base;

    const Point3 corner_f_marker_;
    const Point3 corner_f_world_;

  public:

    /// Construct factor the corner coordinates in the marker and world frames
    TransformFromFactor(const SharedNoiseModel &model, const gtsam::Key &key,
                        const Point3 &corner_f_marker, const Point3 &corner_f_world) :
      Base(model, key), corner_f_marker_{corner_f_marker}, corner_f_world_{corner_f_world}
    {}

    /// evaluate the error
    virtual gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                        boost::optional<gtsam::Matrix &> H = boost::none) const
    {
      return pose.transformFrom(corner_f_marker_, H) - corner_f_world_;
    }
  };

  static void marker_pose_from_corners(const MarkerModel &marker_model,
                                       const std::vector<gtsam::Point3> &corners_f_marker,
                                       double marker_size)
  {
    auto measurement_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(.5, .5, .5));

    /* 1. create graph */
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    /* 2. add measurement factors to the graph */
    for (int i = 0; i < corners_f_marker.size(); i += 1) {
      graph.emplace_shared<TransformFromFactor>(measurement_noise, X(1),
                                               corners_f_marker[i],
                                               marker_model.corners_f_world_[i]);
    }

    /* 3. Create an initial estimate for the camera pose */
    static Pose3 kDeltaPose(Rot3::Rodrigues(-0.1, 0.2, 0.25),
                            Point3(0.05, -0.10, 0.20));
    initial.insert(X(1), marker_model.pose_f_world_.compose(kDeltaPose));

    /* 4. Optimize the graph using Levenberg-Marquardt*/
    auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    auto camera_f_world = result.at<gtsam::Pose3>(X(1));

    gtsam::Marginals marginals(graph, result);
    auto camera_f_world_covariance = marginals.marginalCovariance(X(1));

    std::cout << marker_model.pose_f_world_ << std::endl;
    std::cout << camera_f_world << std::endl << camera_f_world_covariance << std::endl;
  }

  void pfp_marker_pose_from_corners()
  {
    camsim::Model model{camsim::MarkersConfigurations::circle_around_z_axis,
                        camsim::CamerasConfigurations::center_facing_markers,
                        camsim::CameraTypes::distorted_camera};

    for (auto &marker_model : model.markers_.markers_) {
      marker_pose_from_corners(marker_model,
                               model.markers_.corners_f_marker_,
                               model.markers_.marker_size_);
    }
  }
}
