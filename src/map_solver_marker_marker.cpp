
#include "map_run.hpp"

#include "map_solver_runner.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

namespace camsim
{

  class ResectioningFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    const gtsam::Cal3DS2 &cal3ds2_;
    const gtsam::Point3 P_;       ///< 3D point on the calibration rig
    const gtsam::Point2 p_;       ///< 2D measurement of the 3D point

  public:
    /// Construct factor given known point P and its projection p
    ResectioningFactor(const gtsam::SharedNoiseModel &model,
                       const gtsam::Key key,
                       const gtsam::Cal3DS2 &cal3ds2,
                       gtsam::Point2 p,
                       gtsam::Point3 P) :
      NoiseModelFactor1<gtsam::Pose3>(model, key),
      cal3ds2_{cal3ds2},
      P_(std::move(P)),
      p_(std::move(p))
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, cal3ds2_};
      return camera.project(P_, H) - p_;
    }
  };

  class ArucoFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
  {
    const gtsam::Cal3DS2 &cal3ds2_;
    const std::vector<gtsam::Point2> &corners_f_image_;
    const std::vector<gtsam::Point3> &corners_f_marker_;

  public:
    /// Construct factor given known point P and its projection p
    ArucoFactor(const gtsam::SharedNoiseModel &model,
                const gtsam::Key key,
                const gtsam::Cal3DS2 &cal3ds2,
                const std::vector<gtsam::Point2> &corners_f_image,
                const std::vector<gtsam::Point3> &corners_f_marker) :
      NoiseModelFactor1<gtsam::Pose3>(model, key),
      cal3ds2_{cal3ds2},
      corners_f_image_{corners_f_image},
      corners_f_marker_{corners_f_marker}
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                boost::optional<gtsam::Matrix &> H) const override
    {
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{pose, cal3ds2_};

      gtsam::Vector2 p0, p1, p2, p3;
      if (H) {
        gtsam::Matrix26 Dpose0, Dpose1, Dpose2, Dpose3;

        p0 = camera.project(corners_f_marker_[0], Dpose0);
        p1 = camera.project(corners_f_marker_[1], Dpose1);
        p2 = camera.project(corners_f_marker_[2], Dpose2);
        p3 = camera.project(corners_f_marker_[3], Dpose3);

        (*H) = (gtsam::Matrix86{} << Dpose0, Dpose1, Dpose2, Dpose3).finished();

      } else {

        p0 = camera.project(corners_f_marker_[0], boost::none);
        p1 = camera.project(corners_f_marker_[1], boost::none);
        p2 = camera.project(corners_f_marker_[2], boost::none);
        p3 = camera.project(corners_f_marker_[3], boost::none);
      }

//      gtsam::Vector2 e0 = camera.project(corners_f_marker_[0], boost::none) - corners_f_image_[0];
//      gtsam::Vector2 e1 = camera.project(corners_f_marker_[1], boost::none) - corners_f_image_[1];
//      gtsam::Vector2 e2 = camera.project(corners_f_marker_[2], boost::none) - corners_f_image_[2];
//      gtsam::Vector2 e3 = camera.project(corners_f_marker_[3], boost::none) - corners_f_image_[3];

      return (gtsam::Vector8{} << p0 - corners_f_image_[0],
        p1 - corners_f_image_[1],
        p2 - corners_f_image_[2],
        p3 - corners_f_image_[3]).finished();
    }
  };


  class SolverMarkerMarker
  {
    SolverRunner &sr_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};


    PoseWithCovariance calc_camera_f_marker(const std::vector<gtsam::Point2> &corners_f_image,
                                            const gtsam::Pose3 &camera_f_marker_initial)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Add factors to the graph
      for (size_t j = 0; j < corners_f_image.size(); j += 1) {
        graph.emplace_shared<ResectioningFactor>(
          sr_.point2_noise_,
          CameraModel::default_key(), *shared_calibration_,
          corners_f_image[j],
          sr_.model_.markers_.corners_f_marker_[j]);
      }

      // Add the initial estimate for the camera pose in the marker frame
      initial.insert(CameraModel::default_key(), camera_f_marker_initial);

      // Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // return the result
      auto marginals_ptr{sr_.get_marginals(graph, result)};
      return PoseWithCovariance::Extract(result, marginals_ptr.get(), CameraModel::default_key());
    }


    PoseWithCovariance calc_camera_f_marker_aruco(const std::vector<gtsam::Point2> &corners_f_image,
                                                  const gtsam::Pose3 &camera_f_marker_initial)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Add factor to the graph
      graph.emplace_shared<ArucoFactor>(
        sr_.point2x4_noise_,
        CameraModel::default_key(), *shared_calibration_,
        corners_f_image,
        sr_.model_.markers_.corners_f_marker_);


      // Add the initial estimate for the camera pose in the marker frame
      initial.insert(CameraModel::default_key(), camera_f_marker_initial);

      // Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // return the result
      auto marginals_ptr{sr_.get_marginals(graph, result)};
      return PoseWithCovariance::Extract(result, marginals_ptr.get(), CameraModel::default_key());
    }

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". Not quite sure why the RVO doesn't prevent this but it might
      // be because the object is passed by the operator() member and not by the
      // class itself.
      if (graph_.empty()) {
        return;
      }

      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(result) << std::endl;

      auto marginals_slam_ptr{sr_.get_marginals(graph_, result)};
      for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(PoseWithCovariance::Extract(result, marginals_slam_ptr.get(), m.key));
      }
    }

  public:
    explicit SolverMarkerMarker(SolverRunner &sr, bool auto_initial) :
      sr_{sr}, shared_calibration_{boost::make_shared<gtsam::Cal3DS2>(
      sr_.model_.cameras_.calibration_.fx(),
      sr_.model_.cameras_.calibration_.fy(),
      sr_.model_.cameras_.calibration_.skew(),
      sr_.model_.cameras_.calibration_.px(),
      sr_.model_.cameras_.calibration_.py(),
      sr_.model_.cameras_.calibration_.k1(),
      sr_.model_.cameras_.calibration_.k2(),
      sr_.model_.cameras_.calibration_.p1(),
      sr_.model_.cameras_.calibration_.p2())}
    {}

    ~SolverMarkerMarker()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      if (sr_.frames_processed_ == 0) {
        sr_.add_marker_0_prior(graph_);
      }

      auto camera_key{camera.key_};

      for (auto marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        // The marker corners as seen in the image.
        auto corners_f_image = sr_.get_corners_f_images(camera, marker_ref);

        // The measurement
        auto camera_f_marker = calc_camera_f_marker_aruco(corners_f_image,
                                                          sr_.get_perturbed_camera_f_marker(camera, marker_ref));

        std::cout << PoseWithCovariance::to_str(camera_f_marker.pose_) << " "
                  << PoseWithCovariance::to_str(sr_.get_camera_f_marker(camera, marker_ref)) << std::endl;
      }
    }
  };


  std::function<void(const CameraModel &, const std::vector<std::reference_wrapper<const MarkerModel>> &)>
  solver_marker_marker_factory(SolverRunner &sr)
  {
    return SolverMarkerMarker(sr, false);
  }

}
