
#include "map_run.hpp"

#include "map_solver_runner.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

namespace camsim
{

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

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

// ==============================================================================
// SolverProjectBetween class
// ==============================================================================

  class SolverProjectBetween
  {
    SolverRunner &sr_;
    bool initial_with_truth_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};

    std::vector<std::uint64_t> camera_idxs_{};

#if 0
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
#endif

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
      params.setMaxIterations(1000);
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      if (!initial_with_truth_) {

        std::cout << std::endl << "Solve with noisy initial estimates" << std::endl;
        auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
        std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(result) << std::endl;

        auto marginals_slam_ptr{sr_.get_marginals(graph_, result)};
        for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
          sr_.display_results(PoseWithCovariance::Extract(result, marginals_slam_ptr.get(), m.key));
        }

      } else {

        // Solve this with perfect initial values
        gtsam::Values initial;
        for (auto camera_idx : camera_idxs_) {
          auto &camera{sr_.model_.cameras_.cameras_[camera_idx]};
          initial.insert(camera.key_, camera.camera_f_world_);
        }
        for (auto &marker : sr_.model_.markers_.markers_) {
          initial.insert(marker.key_, marker.marker_f_world_);
        }

        std::cout << std::endl << "Solve with perfect initial estimates" << std::endl;
        auto result = gtsam::LevenbergMarquardtOptimizer(graph_, initial, params).optimize();
        std::cout << "initial error = " << graph_.error(initial) << std::endl;
        std::cout << "final error = " << graph_.error(result) << std::endl;

        auto marginals_slam_ptr = sr_.get_marginals(graph_, result);
        for (auto m : result.filter(gtsam::Symbol::ChrTest('m'))) {
          sr_.display_results(PoseWithCovariance::Extract(result, marginals_slam_ptr.get(), m.key));
        }
      }
    }

  public:
    explicit SolverProjectBetween(SolverRunner &sr, bool initial_with_truth) :
      sr_{sr}, initial_with_truth_{initial_with_truth},
      shared_calibration_{boost::make_shared<gtsam::Cal3DS2>(
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

    void operator()(const FrameData &fd)
    {
      auto camera_key{fd.camera_.key_};

      // Add the initial estimate for the camera pose
      initial_.insert(camera_key, fd.camera_f_world_perturbed_);
      camera_idxs_.push_back(fd.camera_.index());

      for (auto &marker_data : fd.marker_datas_) {
        auto marker_key{marker_data.marker_.key_};

        // The marker corners as seen in the image.
        auto &corners_f_image = marker_data.corners_f_image_perturbed_;

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
          initial_.insert(marker_key, marker_data.marker_f_world_perturbed_);
        }
      }
#endif
    }
  };

// ==============================================================================
// SolverProjectBetween class
// ==============================================================================

  class SolverProjectBetweenRepeated
  {
    SolverRunner &sr_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};
    gtsam::Values results_{};

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
    explicit SolverProjectBetweenRepeated(SolverRunner &sr) :
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

    ~SolverProjectBetweenRepeated()
    {
      display_results();
    }

    void operator()(const FrameData &fd)
    {
      auto camera_key{fd.camera_.key_};

      // Add the initial estimate for the camera pose
      initial_.insert(camera_key, fd.camera_f_world_perturbed_);

      for (auto &marker_data : fd.marker_datas_) {
        auto marker_key{marker_data.marker_.key_};

        // The marker corners as seen in the image.
        auto &corners_f_image = marker_data.corners_f_image_perturbed_;

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
          initial_.insert(marker_key, marker_data.marker_f_world_perturbed_);
        }
      }

#if 1
      auto params = gtsam::LevenbergMarquardtParams();
//        params.setVerbosityLM("TERMINATION");
//        params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto results_ = gtsam::LevenbergMarquardtOptimizer(graph_, initial_, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": ";
//        std::cout << "initial error = " << graph_.error(initial_) << std::endl;
      std::cout << "final error = " << graph_.error(results_) << std::endl;

#else
      auto params = gtsam::DoglegParams();
////      params.setVerbosityDL("VERBOSE");
//        params.setVerbosity("TERMINATION");
        params.setRelativeErrorTol(1e-8);
        params.setAbsoluteErrorTol(1e-8);

        results_ = gtsam::DoglegOptimizer(graph_, initial_, params).optimize();
        std::cout << "Frame " << sr_.frames_processed_ << ": ";
//      std::cout << "initial error = " << graph_.error(initial_) << std::endl;
        std::cout << "final error = " << graph_.error(results_) << std::endl;
#endif

      initial_ = results_;
    }
  };

// ==============================================================================
// SolverProjectBetween class
// ==============================================================================

  class SolverProjectBetweenIsam
  {
    SolverRunner &sr_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::ISAM2 isam_{get_isam2_params()};
    std::map<std::uint64_t, std::uint64_t> marker_seen_counts_{};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2DoglegParams dogleg_params{};
      gtsam::ISAM2Params params;
//      params.optimizationParams = dogleg_params;
      params.factorization = gtsam::ISAM2Params::QR;
      params.relinearizeThreshold = 0.01;
      params.relinearizeSkip = 1;
      return params;
    }

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". Not quite sure why the RVO doesn't prevent this but it might
      // be because the object is passed by the operator() member and not by the
      // class itself.
      if (isam_.empty()) {
        return;
      }

      gtsam::Values bestEstimate = isam_.calculateBestEstimate();
      std::cout << "****************************************************" << std::endl;
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      for (auto m : bestEstimate.filter(gtsam::Symbol::ChrTest('m'))) {
        gtsam::Pose3::Jacobian marginal_covariance;
        try {
          marginal_covariance = isam_.marginalCovariance(m.key);
        } catch (gtsam::IndeterminantLinearSystemException &ex) {
          marginal_covariance.setZero();
        }
        sr_.display_results(PoseWithCovariance(m.key, m.value.cast<gtsam::Pose3>(),
                                               marginal_covariance));
      }
    }

  public:
    explicit SolverProjectBetweenIsam(SolverRunner &sr) :
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
      isam_.params().print("isam params");
      auto params = isam_.params();
      int i = 2;
    }

    ~SolverProjectBetweenIsam()
    {
      display_results();
    }

    void operator()(const FrameData &fd)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      if (sr_.frames_processed_ == 0) {
        sr_.add_marker_0_prior(graph, initial);
      }

      auto camera_key{fd.camera_.key_};

      // Add the initial estimate for the camera pose
      initial.insert(camera_key, fd.camera_f_world_perturbed_);

      for (auto &marker_data : fd.marker_datas_) {
        auto marker_key{marker_data.marker_.key_};

        // The marker corners as seen in the image.
        auto &corners_f_image = marker_data.corners_f_image_perturbed_;

        // Add factors to the graph
        for (size_t j = 0; j < corners_f_image.size(); j += 1) {
          graph.emplace_shared<ProjectBetweenFactor>(corners_f_image[j],
                                                     sr_.point2_noise_,
                                                     marker_key,
                                                     sr_.model_.markers_.corners_f_marker_[j],
                                                     camera_key,
                                                     *shared_calibration_);
        }

        // update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_key);
        if (pair == marker_seen_counts_.end()) {
          marker_seen_counts_.insert(std::pair<std::uint64_t, std::uint64_t>{marker_key, 1});
        } else {
          pair->second += 1;
        }

        // Add the initial marker value estimate only if this marker has not been seen.
        if (pair == marker_seen_counts_.end() && !initial.exists(marker_key)) {
          std::cout << "Adding marker " << marker_data.marker_.index()
                    << " at frame " << sr_.frames_processed_ + 1 << std::endl;
          initial.insert(marker_key, marker_data.marker_f_world_perturbed_);
        }
      }

      // Update iSAM with the new factors
      isam_.update(graph, initial);
      isam_.update();
    }
  };


  std::function<void(const FrameData &)>
  solver_project_between_factory(SolverRunner &sr, bool initial_with_truth)
  {
    return SolverProjectBetween(sr, initial_with_truth);
  }

  std::function<void(const FrameData &)>
  solver_project_between_repeated_factory(SolverRunner &sr)
  {
    return SolverProjectBetweenRepeated(sr);
  }

  std::function<void(const FrameData &)>
  solver_project_between_isam_factory(SolverRunner &sr)
  {
    return SolverProjectBetweenIsam(sr);
  }
}
