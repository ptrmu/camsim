
#include "map_run.hpp"

#include "map_solver_runner.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

namespace camsim
{

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    const gtsam::Cal3DS2 &cal3ds2_;
    const gtsam::Point3 point_f_marker_;
    const gtsam::Point2 point_f_image_;

  public:
    ProjectBetweenFactor(gtsam::Point2 point_f_image,
                         const gtsam::SharedNoiseModel &model,
                         const gtsam::Key key_marker,
                         gtsam::Point3 point_f_marker,
                         const gtsam::Key key_camera,
                         const gtsam::Cal3DS2 &cal3ds2) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key_marker, key_camera),
      cal3ds2_{cal3ds2},
      point_f_marker_(std::move(point_f_marker)),
      point_f_image_(std::move(point_f_image))
    {}

    /// evaluate the error
    gtsam::Vector evaluateError(const gtsam::Pose3 &marker_f_world,
                                const gtsam::Pose3 &camera_f_world,
                                boost::optional<gtsam::Matrix &> H1,
                                boost::optional<gtsam::Matrix &> H2) const override
    {
//      (Hp1 || Hp2) ? boost::optional<Matrix&>(D_hx_1P2) : boost::none)
      gtsam::Matrix36 d_point3_wrt_pose3;
      gtsam::Matrix26 d_point2_wrt_pose3;
      gtsam::Matrix23 d_point2_wrt_point3;

      // Transform the point from the Marker frame to the World frame
      gtsam::Point3 point_f_world = marker_f_world.transform_from(
        point_f_marker_,
        H1 ? gtsam::OptionalJacobian<3, 6>(d_point3_wrt_pose3) : boost::none);

      // Project this point to the camera's image frame. Catch and return a default
      // value on a CheiralityException.
      auto camera = gtsam::PinholeCamera<gtsam::Cal3DS2>{camera_f_world, cal3ds2_};
      try {
        gtsam::Point2 point_f_image = camera.project(
          point_f_world,
          H2 ? gtsam::OptionalJacobian<2, 6>(d_point2_wrt_pose3) : boost::none,
          H1 ? gtsam::OptionalJacobian<2, 3>(d_point2_wrt_point3) : boost::none);

        // Return the Jacobian for each input
        if (H1) {
          *H1 = d_point2_wrt_point3 * d_point3_wrt_pose3;
        }
        if (H2) {
          *H2 = d_point2_wrt_pose3;
        }

        // Return the error.
        return point_f_image - point_f_image_;

      } catch (gtsam::CheiralityException &e) {
      }
      if (H1) *H1 = gtsam::Matrix::Zero(2, 6);
      if (H2) *H2 = gtsam::Matrix::Zero(2, 6);
      return gtsam::Vector2::Constant(2.0 * cal3ds2_.fx());
    }
  };


  class SolverProjectBetween
  {
    SolverRunner &sr_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};


    PoseWithCovariance calc_marker_f_world(const std::vector<gtsam::Point2> &corners_f_image,
                                           const gtsam::Pose3 &marker_f_world_initial,
                                           const gtsam::Pose3 &camera_f_world_initial)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Set up the prior for the marker pose
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);

      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(CameraModel::default_key(),
                                                              camera_f_world_initial,
                                                              priorModel);
      initial.insert(CameraModel::default_key(), camera_f_world_initial);

      // Add factors to the graph
      for (size_t j = 0; j < corners_f_image.size(); j += 1) {
        graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                   sr_.point2_noise_,
                                                   MarkerModel::default_key(),
                                                   sr_.model_.markers_.corners_f_marker_[j],
                                                   CameraModel::default_key(),
                                                   *shared_calibration_);
      }

      // Add the initial estimate for the camera poses
      initial.insert(MarkerModel::default_key(), marker_f_world_initial);

      // Optimize the graph using Levenberg-Marquardt
      auto params = gtsam::LevenbergMarquardtParams();
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      // return the result
      auto marginals_ptr{sr_.get_marginals(graph, result)};
      return PoseWithCovariance::Extract(result, marginals_ptr.get(), MarkerModel::default_key());
    }


    PoseWithCovariance calc_camera_f_world(const std::vector<gtsam::Point2> &corners_f_image,
                                           const gtsam::Pose3 &marker_f_world_initial,
                                           const gtsam::Pose3 &camera_f_world_initial)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Set up the prior for the marker pose
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);

      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(MarkerModel::default_key(),
                                                              marker_f_world_initial,
                                                              priorModel);
      initial.insert(MarkerModel::default_key(), marker_f_world_initial);

      // Add factors to the graph
      for (size_t j = 0; j < corners_f_image.size(); j += 1) {
        graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                   sr_.point2_noise_,
                                                   MarkerModel::default_key(),
                                                   sr_.model_.markers_.corners_f_marker_[j],
                                                   CameraModel::default_key(),
                                                   *shared_calibration_);
      }

      // Add the initial estimate for the camera poses
      initial.insert(CameraModel::default_key(), camera_f_world_initial);

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
      if (graph_.size() < 2) {
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
    explicit SolverProjectBetween(SolverRunner &sr, bool auto_initial) :
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
    {
      sr_.add_marker_0_prior(graph_, initial_);
    }

    ~SolverProjectBetween()
    {
      display_results();
    }

#if 0
    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      auto camera_key{camera.key_};

      for (auto &marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        auto marker_f_world_initial = sr_.get_perturbed_marker_f_world(marker_ref);
        auto camera_f_world_initial = sr_.get_perturbed_camera_f_world(camera);

        // The marker corners as seen in the image.
        auto corners_f_image = sr_.get_corners_f_images(camera, marker_ref);

        auto camera_f_world = calc_camera_f_world(corners_f_image,
                                                  marker_ref.get().marker_f_world_,
                                                  camera_f_world_initial);

        auto marker_f_world = calc_marker_f_world(corners_f_image,
                                                  marker_f_world_initial,
                                                  camera.camera_f_world_);

        std::cout << "Camera "
                  << PoseWithCovariance::to_str(camera_f_world.pose_) << " "
                  << PoseWithCovariance::to_str(camera.camera_f_world_) << std::endl;
        std::cout << "Marker "
                  << PoseWithCovariance::to_str(marker_f_world.pose_) << " "
                  << PoseWithCovariance::to_str(marker_ref.get().marker_f_world_) << std::endl;
      }
#else

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      auto camera_key{camera.key_};

      // Add the initial estimate for the camera pose
      auto camera_f_world_initial = sr_.get_perturbed_camera_f_world(camera);
      initial_.insert(camera_key, camera_f_world_initial);

      for (auto &marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        // The marker corners as seen in the image.
        auto corners_f_image = sr_.get_perturbed_corners_f_images(camera, marker_ref);

        // Add factors to the graph
        for (size_t j = 0; j < corners_f_image.size(); j += 1) {
          graph_.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                      sr_.point2_noise_,
                                                      marker_key,
                                                      sr_.model_.markers_.corners_f_marker_[j],
                                                      camera_key,
                                                      *shared_calibration_);
        }

        // Add the initial estimate for the marker pose
        if (!initial_.exists(marker_key)) {
          auto marker_f_world_initial = sr_.get_perturbed_marker_f_world(marker_ref);
          initial_.insert(marker_key, marker_f_world_initial);
        }
      }
#endif
    }
  };


  std::function<void(const CameraModel &, const std::vector<std::reference_wrapper<const MarkerModel>> &)>
  solver_project_between_factory(SolverRunner &sr)
  {
    return SolverProjectBetween(sr, false);
  }
}
