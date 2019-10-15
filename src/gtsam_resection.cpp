
#include "camsim.hpp"

#include "gtsam/inference/Symbol.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <boost/make_shared.hpp>

using namespace gtsam;
using namespace gtsam::noiseModel;
using symbol_shorthand::X;

/**
 * Unary factor on the unknown pose, resulting from meauring the projection of
 * a known 3D point in the image
 */
class ResectioningFactor: public NoiseModelFactor1<Pose3> {
  typedef NoiseModelFactor1<Pose3> Base;

  Cal3_S2::shared_ptr K_; ///< camera's intrinsic parameters
  Point3 P_;              ///< 3D point on the calibration rig
  Point2 p_;              ///< 2D measurement of the 3D point

public:

  /// Construct factor given known point P and its projection p
  ResectioningFactor(const SharedNoiseModel& model, const Key& key,
                     const Cal3_S2::shared_ptr& calib, const Point2& p, const Point3& P) :
    Base(model, key), K_(calib), P_(P), p_(p) {
  }

  /// evaluate the error
  virtual Vector evaluateError(const Pose3& pose, boost::optional<Matrix&> H =
  boost::none) const {
    SimpleCamera camera(pose, *K_);
    return camera.project(P_, H, boost::none, boost::none) - p_;
  }
};

/*******************************************************************************
 * Camera: f = 1, Image: 100x100, center: 50, 50.0
 * Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
 * Known landmarks:
 *    3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * Perfect measurements:
 *    2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
 *******************************************************************************/

int gtsam_resection()
{
  /* read camera intrinsic parameters */
  Cal3_S2::shared_ptr calib(new Cal3_S2(1, 1, 0, 50, 50));

  /* 1. create graph */
  NonlinearFactorGraph graph;

  /* 2. add factors to the graph */
  // add measurement factors
  SharedDiagonal measurementNoise = Diagonal::Sigmas(Vector2(5., .5));
  boost::shared_ptr<ResectioningFactor> factor;
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                           Point2(55, 45), Point3(10, 10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                           Point2(45, 45), Point3(-10, 10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                           Point2(45, 55), Point3(-10, -10, 0));
  graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib,
                                           Point2(55, 55), Point3(10, -10, 0));

  /* 3. Create an initial estimate for the camera pose */
  Values initial;
  initial.insert(X(1),
//                 Pose3(Rot3(1, 1, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 2)));
                 Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 2)));

  graph.print("full graph");

  /* 4. Optimize the graph using Levenberg-Marquardt*/
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final result:\n");

  std::cout.precision(2);
  Marginals marginals(graph, result);
  std::cout << "x1 covariance:\n" << marginals.marginalCovariance(X(1)) << std::endl;

  std::cout << "initial error = " << graph.error(initial) << std::endl;
  std::cout << "final error = " << graph.error(result) << std::endl;

  return 0;

}
