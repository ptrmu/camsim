
#include "map_run.hpp"

#include "map_solver_runner.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <opencv2/opencv.hpp>

namespace camsim
{
// ==============================================================================
// ConvertPoints class
// ==============================================================================

  struct ConvertPoints
  {
    template<class TPoint>
    static TPoint to_point(const gtsam::Point2 &point2)
    {
      return TPoint{point2.x(),
                    point2.y()};
    }

    template<class TPoint>
    static TPoint to_point(const gtsam::Point3 &point3)
    {
      return TPoint{point3.x(),
                    point3.y(),
                    point3.z()};
    }

    template<class TPointOut, class TPointIn>
    static std::vector<TPointOut> points_to_points(const std::vector<TPointIn> &points_in)
    {
      std::vector<TPointOut> points_out;
      for (auto &point_in : points_in) {
        points_out.emplace_back(to_point<TPointOut>(point_in));
      }
      return points_out;
    }
  };

// ==============================================================================
// ProjectBetweenFactor class
// ==============================================================================

  class ProjectBetweenFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
  {
    const gtsam::Cal3DS2 &cal3ds2_;
    const gtsam::Point3 corner_f_marker_;
    const gtsam::Point2 corner_f_image_;

  public:
    ProjectBetweenFactor(gtsam::Point2 corner_f_image,
                         const gtsam::SharedNoiseModel &model,
                         gtsam::Key marker_key,
                         gtsam::Point3 corner_f_marker,
                         gtsam::Key camera_key,
                         const gtsam::Cal3DS2 &cal3ds2) :
      NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, marker_key, camera_key),
      cal3ds2_{cal3ds2},
      corner_f_marker_(std::move(corner_f_marker)),
      corner_f_image_(std::move(corner_f_image))
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
        corner_f_marker_,
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
        return point_f_image - corner_f_image_;

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
// SolverProjectBetweenRepeated class
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
// SolverProjectBetweenIsam class
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
    {}

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

// ==============================================================================
// SolverProjectBetweenOpencv class
// ==============================================================================

  class SolverProjectBetweenOpencv
  {
    SolverRunner &sr_;
    const gtsam::Cal3DS2 &cal_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    cv::Mat cv_camera_matrix_;
    cv::Mat cv_dist_coeffs_;
    std::vector<cv::Point3d> cv_corners_f_marker_;

    gtsam::ISAM2 isam_{get_isam2_params()};
    std::map<std::uint64_t, std::uint64_t> marker_seen_counts_{};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params params;
      params.factorization = gtsam::ISAM2Params::QR;
      params.relinearizeThreshold = 0.01;
      params.relinearizeSkip = 1;
      return params;
    }

    PoseWithCovariance cv_calc_camera_f_marker(const std::vector<gtsam::Point2> &corners_f_image)
    {
      cv::Vec3d rvec, tvec;
      auto cv_corners_f_image{ConvertPoints::points_to_points<cv::Point2d, gtsam::Point2>(corners_f_image)};

      // Solve for pose
      cv::solvePnP(cv_corners_f_marker_, cv_corners_f_image,
                   cv_camera_matrix_, cv_dist_coeffs_,
                   rvec, tvec);

      // Convert the result to gtsam world
      gtsam::Point3 t(tvec[0], tvec[1], tvec[2]);
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      gtsam::Matrix3 m;
      for (int row = 0; row < 3; row++) {
        for (int col = 0; col < 3; col++) {
          m(row, col) = rmat.at<double>(row, col);  // Row- vs. column-major order
        }
      }

      // return the result. Invert the transform to get camera_f_marker instead of marker_f_camera
      return PoseWithCovariance{CameraModel::default_key(), gtsam::Pose3{gtsam::Rot3{m}, t}.inverse(), gtsam::Z_6x6};
    }


    void add_project_between_factors(gtsam::NonlinearFactorGraph &graph,
                                     gtsam::Key camera_key,
                                     const std::vector<MarkerData> &marker_datas,
                                     std::function<bool(bool, const MarkerData &)> do_add_func)
    {
      for (auto &marker_data : marker_datas) {
        auto marker_key{marker_data.marker_.key_};

        // Look for the marker_seen_count record for this marker.
        auto pair = marker_seen_counts_.find(marker_key);

        // Check that we should add the measurement for this marker.
        if (do_add_func(pair != marker_seen_counts_.end(), marker_data)) {

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
          if (pair == marker_seen_counts_.end()) {
            marker_seen_counts_.insert(std::pair<std::uint64_t, std::uint64_t>{marker_key, 1});
          } else {
            pair->second += 1;
          }
        }
      }
    }

    std::uint64_t good_marker_seen_counts_{0};
    bool good_marker_is_fixed_{false};
    const MarkerData *good_marker_data_{nullptr};

    void good_marker_start()
    {
      good_marker_seen_counts_ = 0;
      good_marker_is_fixed_ = false;
      good_marker_data_ = nullptr;
    }

    void good_marker_check(const MarkerData &marker_data)
    {
      if (good_marker_is_fixed_) {
        return;
      }

      // The zero'th marker is the fixed marker and it will always be best
      if (marker_data.marker_.key_ == sr_.model_.markers_.markers_[0].key_) {
        good_marker_is_fixed_ = true;
        good_marker_data_ = &marker_data;
        return;
      }

      // Otherwise the best marker is the one that has been seen the most.
      auto pair = marker_seen_counts_.find(marker_data.marker_.key_);
      assert(pair != marker_seen_counts_.end()); // This should always be found
      if (pair->second > good_marker_seen_counts_) {
        good_marker_data_ = &marker_data;
        good_marker_seen_counts_ = pair->second;
      }
    }

    const MarkerData *good_marker()
    {
      return good_marker_data_;
    }

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". Not quite sure why the RVO doesn't prevent this but it might
      // be because the object is passed by the operator() member and not by the
      // class itself.
      if (isam_.size() < 2) {
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
    explicit SolverProjectBetweenOpencv(SolverRunner &sr) :
      sr_{sr}, cal_{sr_.model_.cameras_.calibration_},
      shared_calibration_{boost::make_shared<gtsam::Cal3DS2>(
        cal_.fx(), cal_.fy(), cal_.skew(),
        cal_.px(), cal_.py(),
        cal_.k1(), cal_.k2(),
        cal_.p1(), cal_.p2())},
      cv_camera_matrix_{(cv::Mat_<double>(3, 3)
        << cal_.fx(), cal_.skew(), cal_.px(),
        0, cal_.fy(), cal_.py(),
        0, 0, 1)},
      cv_dist_coeffs_{(cv::Mat_<double>(4, 1)
        << cal_.k1(), cal_.k2(),
        cal_.p1(), cal_.p2())},
      cv_corners_f_marker_{
        ConvertPoints::points_to_points<cv::Point3d, gtsam::Point3>(sr_.model_.markers_.corners_f_marker_)}
    {
      // Initialize the isam with the fixed prior
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      // Add the fixed marker to the graph.
      sr_.add_marker_0_prior(graph, initial);
      marker_seen_counts_.insert(std::pair<std::uint64_t, std::uint64_t>{sr_.model_.markers_.markers_[0].key_, 1});

      isam_.update(graph, initial);
    }

    ~SolverProjectBetweenOpencv()
    {
      display_results();
    }

    void operator()(const FrameData &fd)
    {
      auto camera_key{fd.camera_.key_};
      bool unknown_exist{false};

      { // First pass through the markers for those that have been seen already
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Prepare for the best marker search.
        good_marker_start();

        auto do_func = [this, &unknown_exist](bool known_marker, const MarkerData &marker_data) -> bool
        {
          if (known_marker) {
            good_marker_check(marker_data);
          } else {
            unknown_exist = true;
          }
          return known_marker;
        };

        add_project_between_factors(graph, camera_key, fd.marker_datas_, do_func);

        // If there is no good marker (if there are no known markers) then just return.
        if (good_marker() == nullptr) {
          return;
        }

        // Find the camera pose relative to a good marker using the image points
        auto cv_camera_f_best_marker = cv_calc_camera_f_marker(good_marker()->corners_f_image_);

        // Get the latest estimate of the good marker location from the isam solver
        auto best_marker_f_world = isam_.calculateEstimate<gtsam::Pose3>(good_marker()->marker_.key_);

        // Calculate the good estimate of camera_f_world and set as the initial value.
        auto camera_f_world_good = best_marker_f_world * cv_camera_f_best_marker.pose_.inverse();
        initial.insert(camera_key, camera_f_world_good);

        // Update iSAM with the factors for known markers.
        isam_.update(graph, initial);
        isam_.update();
      }

      if (unknown_exist) {
        // Second pass through the markers for those that have not been seen yet
        gtsam::NonlinearFactorGraph graph{};
        gtsam::Values initial{};

        // Get the latest estimate of the camera location
        auto camera_f_world_latest = isam_.calculateEstimate<gtsam::Pose3>(camera_key);

        auto do_func = [this, &initial, camera_f_world_latest](bool known_marker,
                                                               const MarkerData &marker_data) -> bool
        {
          if (!known_marker && !initial.exists(marker_data.marker_.key_)) {

            auto cv_camera_f_marker = cv_calc_camera_f_marker(marker_data.corners_f_image_);
            auto marker_f_world = camera_f_world_latest * cv_camera_f_marker.pose_.inverse();
            initial.insert(marker_data.marker_.key_, marker_f_world);

            std::cout << "Added marker " << marker_data.marker_.index()
                      << " at frame " << sr_.frames_processed_ + 1 << " "
                      << PoseWithCovariance::to_str(marker_f_world) << std::endl;
          }
          return !known_marker;
        };

        add_project_between_factors(graph, camera_key, fd.marker_datas_, do_func);

        // Update iSAM with the new factors to unknown markers
        isam_.update(graph, initial);
        isam_.update();
      }
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

  std::function<void(const FrameData &)>
  solver_project_between_opencv_factory(SolverRunner &sr)
  {
    return SolverProjectBetweenOpencv(sr);
  }

}
