

#include "map_run.hpp"
#include "model.hpp"
#include "pose_with_covariance.hpp"

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include "gtsam/inference/Symbol.h"
#include "gtsam/linear/Sampler.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/InitializePose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>

namespace camsim
{
  typedef std::reference_wrapper<const MarkerModel> MarkerModelRef;

  struct SolverRunner
  {
    typedef std::function<void(const CameraModel &,
                               const std::vector<MarkerModelRef> &)> AddFrameMeasurementsFunc;

    typedef std::function<AddFrameMeasurementsFunc(SolverRunner &)> SolverFactoryFunc;

    const Model &model_;
    const gtsam::Vector6 pose3_sampler_sigmas_;
    const gtsam::Vector2 point2_sampler_sigmas_;
    const bool print_covariance_;

    const gtsam::SharedNoiseModel pose3_noise_;
    const gtsam::SharedNoiseModel point2_noise_;

    gtsam::Sampler pose3_sampler_{};
    gtsam::Sampler point2_sampler_{};
    int frames_processed_{0};

    SolverRunner(const Model &model,
                 const gtsam::Vector6 &pose3_sampler_sigmas,
                 const gtsam::Vector6 &pose3_noise_sigmas,
                 const gtsam::Vector2 &point2_sampler_sigmas,
                 const gtsam::Vector2 &point2_noise_sigmas,
                 bool print_covariance) :
      model_{model},
      pose3_sampler_sigmas_{std::move(pose3_sampler_sigmas)},
      point2_sampler_sigmas_{std::move(point2_sampler_sigmas)},
      print_covariance_{print_covariance},
      pose3_noise_{gtsam::noiseModel::Diagonal::Sigmas(pose3_noise_sigmas)},
      point2_noise_{gtsam::noiseModel::Diagonal::Sigmas(point2_noise_sigmas)}
    {}

    void operator()(const SolverFactoryFunc &solver_factory)
    {
      // Prepare
      pose3_sampler_ = gtsam::Sampler{pose3_sampler_sigmas_, 42u};
      point2_sampler_ = gtsam::Sampler{point2_sampler_sigmas_, 42u};
      frames_processed_ = 0;

      // Instantiate the solver
      auto solver{solver_factory(*this)};

      // Loop over all the cameras
      for (auto &camera : model_.cameras_.cameras_) {
        std::vector<MarkerModelRef> marker_refs{};

        // Figure out which markers are visible from this camera
        for (auto &marker : model_.markers_.markers_) {
          if (!model_.corners_f_images_[camera.index()][marker.index()].corners_f_image_.empty()) {
            marker_refs.emplace_back(MarkerModelRef{marker});
          }
        }

        // Let the solver work on these measurements.
        if (marker_refs.size() > 1) {
          solver(camera, marker_refs);
          frames_processed_ += 1;
        }
      }
    }

    void display_results(const PoseWithCovariance &pose_f_world) const
    {
      gtsam::Symbol sym{pose_f_world.key_};
      std::cout << sym.chr() << sym.index() << PoseWithCovariance::to_str(pose_f_world.pose_)
                << " std: " << PoseWithCovariance::to_stddev_str(pose_f_world.cov_)
                << std::endl;
      if (print_covariance_) {
        std::cout << PoseWithCovariance::to_str(pose_f_world.cov_) << std::endl;
      }
    }

    void display_results(const std::array<camsim::PointWithCovariance, 4> &corners_f_world) const
    {
      std::cout << "m" << gtsam::Symbol(corners_f_world[0].key_).index() << std::endl;
      for (auto const &corner_f_world : corners_f_world) {
        std::cout << PointWithCovariance::to_str(corner_f_world.point_)
                  << " std: " << PointWithCovariance::to_stddev_str(corner_f_world.cov_)
                  << std::endl;
        if (print_covariance_) {
          std::cout << PointWithCovariance::to_str(corner_f_world.cov_) << std::endl;
        }
      }
    }

    gtsam::Pose3 get_perturbed_camera_f_world(const CameraModel &camera)
    {
      return camera.camera_f_world_.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_perturbed_marker_f_world(const MarkerModel &marker)
    {
      return marker.marker_f_world_.retract(pose3_sampler_.sample());
    }

    gtsam::Pose3 get_camera_f_marker(const CameraModel &camera,
                                     const MarkerModel &marker)
    {
      return (marker.marker_f_world_.inverse() * camera.camera_f_world_);
    }

    gtsam::Pose3 get_perturbed_camera_f_marker(const CameraModel &camera,
                                               const MarkerModel &marker)
    {
      return get_camera_f_marker(camera, marker).retract(pose3_sampler_.sample());
    }

    std::vector<gtsam::Point2> get_perturbed_corners_f_images(const CameraModel &camera,
                                                              const MarkerModel &marker)
    {
      auto corners_f_image{model_.corners_f_images_[camera.index()][marker.index()].corners_f_image_};
      for (auto &corner_f_image : corners_f_image) {
        corner_f_image = corner_f_image.retract(point2_sampler_.sample());
      }
      return corners_f_image;
    }

    void add_marker_0_prior(gtsam::NonlinearFactorGraph &graph)
    {
      // Add the prior for marker 0
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_6x1);
      graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(model_.markers_.markers_[0].key_,
                                                              model_.markers_.markers_[0].marker_f_world_,
                                                              priorModel);
    }

    std::unique_ptr<gtsam::Marginals> get_marginals(gtsam::NonlinearFactorGraph &graph, gtsam::Values &values)
    {
      try {
        return std::make_unique<gtsam::Marginals>(graph, values);
      }
      catch (gtsam::IndeterminantLinearSystemException &ex) {
      }
      std::cout << "Could not create marginals, trying QR" << std::endl;

      try {
        return std::make_unique<gtsam::Marginals>(graph, values, gtsam::Marginals::QR);
      }
      catch (gtsam::IndeterminantLinearSystemException &ex) {
      }
      std::cout << "Could not create marginals, returning null" << std::endl;

      return std::unique_ptr<gtsam::Marginals>{};
    }
  };

  class SolverBatch
  {
    SolverRunner &sr_;
    const bool auto_initial_;

    gtsam::NonlinearFactorGraph graph_{};
    gtsam::Values initial_{};

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved". Not quite sure why the RVO doesn't prevent this but it might
      // be because the object is passed by the operator() member and not by the
      // class itself.
      if (graph_.empty()) {
        return;
      }

      if (auto_initial_) {
        initial_ = gtsam::InitializePose3::initialize(graph_);
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
    explicit SolverBatch(SolverRunner &sr, bool auto_initial) :
      sr_{sr}, auto_initial_{auto_initial}
    {}

    ~SolverBatch()
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

      // Add the initial camera pose estimate
      if (!auto_initial_) {
        initial_.insert(camera_key, sr_.get_perturbed_camera_f_world(camera));
      }

      for (auto marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        // Add the measurement factor.
        graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          sr_.get_perturbed_camera_f_marker(camera, marker_ref),
          sr_.pose3_noise_);

        // Add the initial marker value estimate
        if (!auto_initial_ && !initial_.exists(marker_key)) {
          initial_.insert(marker_key, sr_.get_perturbed_marker_f_world(marker_ref));
        }
      }
    }
  };

  class SolverBatchSFM
  {
    SolverRunner &sr_;
    const boost::shared_ptr<gtsam::Cal3DS2> shared_calibration_;

    gtsam::NonlinearFactorGraph graph_slam_{};
    gtsam::NonlinearFactorGraph graph_sfm_{};

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

    class TransformFromFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3>
    {
      const gtsam::Point3 corner_f_marker_;
      const gtsam::Point3 corner_f_world_;

    public:

      /// Construct factor the corner coordinates in the marker and world frames
      TransformFromFactor(const gtsam::SharedNoiseModel &model, const gtsam::Key &key,
                          const gtsam::Point3 &corner_f_marker, const gtsam::Point3 &corner_f_world) :
        gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
        corner_f_marker_{corner_f_marker},
        corner_f_world_{corner_f_world}
      {}

      /// evaluate the error
      virtual gtsam::Vector evaluateError(const gtsam::Pose3 &pose,
                                          boost::optional<gtsam::Matrix &> H = boost::none) const
      {
        return pose.transformFrom(corner_f_marker_, H) - corner_f_world_;
      }
    };

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

    PoseWithCovariance calc_marker_f_world(const PointWithCovariance::FourPoints &corners_f_world)
    {
      auto marker_key{MarkerModel::marker_key_from_corner_key(corners_f_world[0].key_)};

      /* Create graph */
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      /* Add measurement factors to the graph */
      for (std::size_t j = 0; j < corners_f_world.size(); j += 1) {
        auto const &corner_f_world = corners_f_world[j];
        graph.emplace_shared<TransformFromFactor>(
          gtsam::noiseModel::Gaussian::Covariance(corner_f_world.cov_), marker_key,
          sr_.model_.markers_.corners_f_marker_[j], corner_f_world.point_);
      }

      /* 3. Create an initial estimate for the camera pose */
      auto &cfw = corners_f_world;
      auto t = (cfw[0].point_ + cfw[1].point_ + cfw[2].point_ + cfw[3].point_) / 4.;
      auto x_axis = ((cfw[1].point_ + cfw[2].point_) / 2. - t).normalized();
      auto z_axis = x_axis.cross(cfw[1].point_ - t).normalized();
      auto y_axis = z_axis.cross(x_axis);
      auto r = gtsam::Rot3{(gtsam::Matrix3{} << x_axis, y_axis, z_axis).finished()};
      auto camera_f_world_initial = gtsam::Pose3{r, t};
      initial.insert(marker_key, camera_f_world_initial);

      /* 4. Optimize the graph using Levenberg-Marquardt*/
      auto params = gtsam::LevenbergMarquardtParams();
//      params.setVerbosityLM("TERMINATION");
//      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result = gtsam::LevenbergMarquardtOptimizer(graph, initial, params).optimize();
//      std::cout << "initial error = " << graph.error(initial) << std::endl;
//      std::cout << "final error = " << graph.error(result) << std::endl;

      auto camera_f_world = result.at<gtsam::Pose3>(marker_key);
      gtsam::Marginals marginals(graph, result);
      gtsam::Matrix6 camera_f_world_covariance = marginals.marginalCovariance(marker_key);

      return PoseWithCovariance(marker_key, camera_f_world, camera_f_world_covariance);
    }

    void display_results()
    {
      // These objects get copy constructed and will sometimes get destructed without
      // being "solved".
      if (graph_slam_.empty()) {
        return;
      }

      // Initialize the pose3 graph auto-magically.
      auto initial_slam = gtsam::InitializePose3::initialize(graph_slam_);

      auto params = gtsam::LevenbergMarquardtParams();
      params.setVerbosityLM("TERMINATION");
      params.setVerbosity("TERMINATION");
      params.setRelativeErrorTol(1e-8);
      params.setAbsoluteErrorTol(1e-8);

      auto result_slam = gtsam::LevenbergMarquardtOptimizer(graph_slam_, initial_slam, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_slam_.error(initial_slam) << std::endl;
      std::cout << "final error = " << graph_slam_.error(result_slam) << std::endl;

      auto marginals_slam_ptr{sr_.get_marginals(graph_slam_, result_slam)};
      for (auto m : result_slam.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(PoseWithCovariance::Extract(result_slam, marginals_slam_ptr.get(), m.key));
      }

      // Take initial_slam (or result_slam) and initialize initial_sfm
      gtsam::Values initial_sfm{};
      // First do the cameras
      for (auto const &c : result_slam.filter(gtsam::Symbol::ChrTest('c'))) {
        initial_sfm.insert(c.key, c.value.cast<gtsam::Pose3>());
      }
      // Take the marker poses in the world frame and generate locations of the corner points.
      for (auto const &m : result_slam.filter(gtsam::Symbol::ChrTest('m'))) {
        auto marker_key{m.key};
        auto t_world_marker{m.value.cast<gtsam::Pose3>()};
        for (size_t j = 0; j < sr_.model_.markers_.corners_f_marker_.size(); j += 1) {
          initial_sfm.insert(CornerModel::corner_key(marker_key, j),
                             t_world_marker * sr_.model_.markers_.corners_f_marker_[j]);
        }
      }

      params.maxIterations = 1000;
      auto result_sfm = gtsam::LevenbergMarquardtOptimizer(graph_sfm_, initial_sfm, params).optimize();
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      std::cout << "initial error = " << graph_sfm_.error(initial_sfm) << std::endl;
      std::cout << "final error = " << graph_sfm_.error(result_sfm) << std::endl;

      auto marginals_sfm_ptr{sr_.get_marginals(graph_sfm_, result_sfm)};
      std::vector<PointWithCovariance::FourPoints> corner_points_list{};
      for (auto const &m : result_slam.filter(gtsam::Symbol::ChrTest('m'))) {
        corner_points_list.emplace_back(PointWithCovariance::Extract4(result_sfm, marginals_sfm_ptr.get(), m.key));
      }

      for (auto const &corner_points : corner_points_list) {
        sr_.display_results(corner_points);
      }

      for (auto const &corner_points : corner_points_list) {
        auto marker_f_world{calc_marker_f_world(corner_points)};
        sr_.display_results(marker_f_world);
      }
    }

    void add_marker_0_corner_priors()
    {
      auto marker_key{MarkerModel::default_key()};
      auto t_world_marker{sr_.model_.markers_.markers_[0].marker_f_world_};
      static auto priorModel = gtsam::noiseModel::Constrained::MixedSigmas(gtsam::Z_3x1);
      for (size_t j = 0; j < sr_.model_.markers_.corners_f_marker_.size(); j += 1) {
        graph_sfm_.emplace_shared<gtsam::PriorFactor<gtsam::Point3> >(
          CornerModel::corner_key(marker_key, j),
          t_world_marker * sr_.model_.markers_.corners_f_marker_[j],
          priorModel);
      }
    }

  public:
    explicit SolverBatchSFM(SolverRunner &sr) :
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

    ~SolverBatchSFM()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      if (sr_.frames_processed_ == 0) {
        sr_.add_marker_0_prior(graph_slam_);
        add_marker_0_corner_priors();
      }

      auto camera_key{camera.key_};

      for (auto &marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        // The marker corners as seen in the image.
        auto corners_f_image = sr_.get_perturbed_corners_f_images(camera, marker_ref);

        // The slam measurement
        auto camera_f_marker = calc_camera_f_marker(corners_f_image, sr_.get_camera_f_marker(camera, marker_ref));

        // Add the measurement factor for slam.
        graph_slam_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          camera_f_marker.pose_, sr_.pose3_noise_);

        // Add the measurement factor for sfm as a general projection factor
        for (size_t j = 0; j < corners_f_image.size(); j += 1) {
          auto corner_key{CornerModel::corner_key(marker_key, j)};
          graph_sfm_.emplace_shared<gtsam::GenericProjectionFactor<gtsam::Pose3, gtsam::Point3, gtsam::Cal3DS2>>(
            corners_f_image[j], sr_.point2_noise_,
            camera_key, corner_key,
            shared_calibration_);
        }
      }
    }
  };

  class SolverISAM
  {
    SolverRunner &sr_;

    gtsam::ISAM2 isam2_{get_isam2_params()};
    std::map<std::uint64_t, int> marker_seen_counts_{};

    static gtsam::ISAM2Params get_isam2_params()
    {
      gtsam::ISAM2Params isam2_params;
      isam2_params.relinearizeThreshold = 0.01;
      isam2_params.relinearizeSkip = 1;
      return isam2_params;
    }

    void display_results()
    {
      if (isam2_.empty()) {
        return;
      }

      gtsam::Values currentEstimate = isam2_.calculateEstimate();
      std::cout << "****************************************************" << std::endl;
      std::cout << "Frame " << sr_.frames_processed_ << ": " << std::endl;
      for (auto m : currentEstimate.filter(gtsam::Symbol::ChrTest('m'))) {
        sr_.display_results(PoseWithCovariance(m.key, m.value.cast<gtsam::Pose3>(),
                                               isam2_.marginalCovariance(m.key)));
      }
    }

  public:
    explicit SolverISAM(SolverRunner &sr) :
      sr_{sr}
    {}

    ~SolverISAM()
    {
      display_results();
    }

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      gtsam::NonlinearFactorGraph graph{};
      gtsam::Values initial{};

      if (sr_.frames_processed_ == 0) {
        sr_.add_marker_0_prior(graph);
      }

      auto camera_key{camera.key_};

      // Add the initial camera pose estimate
      initial.insert(camera_key, sr_.get_perturbed_camera_f_world(camera));

      for (auto &marker_ref : marker_refs) {
        auto marker_key{marker_ref.get().key_};

        // Add the measurement factor.
        graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
          marker_key, camera_key,
          sr_.get_perturbed_camera_f_marker(camera, marker_ref), sr_.pose3_noise_);

        // update the marker seen counts
        auto pair = marker_seen_counts_.find(marker_ref.get().key_);
        if (pair == marker_seen_counts_.end()) {
          marker_seen_counts_.insert(std::pair<std::uint64_t, int>{marker_ref.get().key_, 1});
        } else {
          pair->second += 1;
        }

        // Add the initial marker value estimate only if this marker has not been seen.
        if (pair == marker_seen_counts_.end()) {
          std::cout << "Adding marker " << marker_ref.get().index()
                    << " at frame " << sr_.frames_processed_ + 1 << std::endl;
          initial.insert(marker_key, sr_.get_perturbed_marker_f_world(marker_ref));
        }
      }

      // Update iSAM with the new factors
      isam2_.update(graph, initial);
      isam2_.update();
    }
  };


  void map_global(double r_sigma,
                  double t_sigma,
                  double u_sampler_sigma,
                  double u_noise_sigma)
  {
    int n_markers = 8;
    int n_cameras = 1024;

//    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingOrigin{n_markers, 2.},
//                            PoseGens::SpinAboutZAtOriginFacingOut{n_cameras},
//                            camsim::CameraTypes::simulation,
//                            0.1775}};

    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingAlongZ{n_markers, 2., 2., false},
                            PoseGens::CircleInXYPlaneFacingAlongZ{n_cameras, 2., 0., true},
                            camsim::CameraTypes::simulation,
                            0.1775}};

//    model.print_corners_f_image();

//    double r_sigma = 0.1;
//    double t_sigma = 0.3;
//    double u_sigma = 0.5;

    SolverRunner solver_runner{model,
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                 gtsam::Vector3::Constant(t_sigma)).finished(),
                               (gtsam::Vector6{} << gtsam::Vector3::Constant(r_sigma),
                                 gtsam::Vector3::Constant(t_sigma)).finished(),
                               gtsam::Vector2::Constant(u_sampler_sigma),
                               gtsam::Vector2::Constant(u_noise_sigma),
                               false};

//    solver_runner([](SolverRunner &solver_runner)
//                  { return SolverBatch{solver_runner, false}; });
//
//    solver_runner([](SolverRunner &solver_runner)
//                  { return SolverBatch{solver_runner, true}; });
//
//    solver_runner([](SolverRunner &solver_runner)
//                  { return SolverISAM{solver_runner}; });

    solver_runner([](SolverRunner &solver_runner)
                  { return SolverBatchSFM{solver_runner}; });
  }
}
