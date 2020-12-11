
#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include "../model.hpp"
#include "pfp_run.hpp"
#include "../pose_with_covariance.hpp"

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
    auto &cfw = marker_model.corners_f_world_;
    auto t = (cfw[0] + cfw[1] + cfw[2] + cfw[3]) / 4.;
    auto x_axis = ((cfw[1] + cfw[2]) / 2. - t).normalized();
    auto z_axis = x_axis.cross(cfw[1] - t).normalized();
    auto y_axis = z_axis.cross(x_axis);
    auto r = gtsam::Rot3{(gtsam::Matrix3{} << x_axis, y_axis, z_axis).finished()};
    auto camera_f_world_initial = gtsam::Pose3{r, t};
    initial.insert(X(1), camera_f_world_initial);

    /* 4. Optimize the graph using Levenberg-Marquardt*/
    auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    auto camera_f_world = result.at<gtsam::Pose3>(X(1));

    gtsam::Marginals marginals(graph, result);
    gtsam::Matrix6 camera_f_world_covariance = marginals.marginalCovariance(X(1));

    std::cout << "Marker " << marker_model.index() << std::endl;
    std::cout << "Truth " << PoseWithCovariance::to_str(marker_model.marker_f_world_) << std::endl;
    std::cout << "Init  " << PoseWithCovariance::to_str(camera_f_world_initial) << std::endl;
    std::cout << "Found " << PoseWithCovariance::to_str(camera_f_world) << std::endl;
    std::cout << PoseWithCovariance::to_matrix_str(camera_f_world_covariance, true) << std::endl;
  }

  int pfp_marker_pose_from_corners()
  {
    camsim::Model model{ModelConfig{camsim::MarkersConfigurations::tetrahedron,
                                    camsim::CamerasConfigurations::z2_facing_origin,
                                    camsim::CameraTypes::distorted_camera}};

    for (auto &marker_model : model.markers_.markers_) {
      marker_pose_from_corners(marker_model,
                               model.markers_.corners_f_marker_,
                               model.markers_.cfg_.marker_size_);
    }

    return 0;
  }
}
