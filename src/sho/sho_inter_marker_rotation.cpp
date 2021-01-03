#pragma ide diagnostic ignored "modernize-use-nodiscard"
#pragma ide diagnostic ignored "NotImplementedFunctions"
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
#pragma ide diagnostic ignored "OCUnusedStructInspection"
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection"

#include "sho_run.hpp"

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/camera_info.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "gtsam/geometry/Pose3.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <opencv2/opencv.hpp>

namespace camsim
{
  class MarkerMeasurement
  {
    std::uint64_t id_;
    fvlam::Observation observation_;
    fvlam::Transform3 marker_f_camera_;

  public:
    MarkerMeasurement(std::uint64_t id,
                      fvlam::Observation observation,
                      fvlam::Transform3 marker_r_camera) :
      id_{id}, observation_{std::move(observation)}, marker_f_camera_{std::move(marker_r_camera)}
    {}

    const auto &id() const
    { return id_; }

    const auto &observation() const
    { return observation_; }

    const auto &marker_f_camera() const
    { return marker_f_camera_; }
  };

  class ImageMeasurement
  {
    std::uint64_t stamp_;
    fvlam::CameraInfo camera_info_;
    std::vector<MarkerMeasurement> marker_measurements_{};

  public:
    ImageMeasurement(std::uint64_t stamp,
                     fvlam::CameraInfo camera_info,
                     std::vector<MarkerMeasurement> marker_measurements) :
      stamp_{stamp}, camera_info_{std::move(camera_info)}, marker_measurements_{std::move(marker_measurements)}
    {}

    const auto &stamp() const
    { return stamp_; }

    const auto &camera_info() const
    { return camera_info_; }

    const auto &measurements() const
    { return marker_measurements_; }
  };
}

namespace fvlam
{
  template<>
  Observation Observation::from<const camsim::MarkerMeasurement>(
    const camsim::MarkerMeasurement &other)
  {
    return other.observation();
  }

  template<>
  Observations Observations::from<const std::vector<camsim::MarkerMeasurement>>(
    const std::vector<camsim::MarkerMeasurement> &other)
  {
    Observations observations{};

    for (auto &measurement : other) {
      auto mo{Observation::from(measurement)};
      observations.add(mo);
    }

    return observations;
  }
}

namespace camsim
{

  /* Calculate the transform from one marker to another given
   * their transforms from a camera. The optimization is done
   * here so the error can be propagated. There is probably a
   * way to do this analytically but I don't know how.
   */
  fvlam::Transform3WithCovariance calc_t_marker0_marker1(
    const fvlam::Transform3WithCovariance &t_camera_marker0,
    const fvlam::Transform3WithCovariance &t_camera_marker1)
  {
    static const std::uint64_t camera_key = 0;
    static const std::uint64_t marker0_key = 1;
    static const std::uint64_t marker1_key = 2;

    // Prepare for an optimization
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // The keys are used as
    // A prior on m0 has no uncertainty.
    auto m0_f_w_covariance = gtsam::noiseModel::Constrained::All(gtsam::Pose3::dimension);
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3> >(
      marker0_key, gtsam::Pose3(), m0_f_w_covariance);

    // Add the measurements
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      camera_key, marker0_key,
      t_camera_marker0.tf().to<gtsam::Pose3>(),
      gtsam::noiseModel::Gaussian::Covariance(t_camera_marker0.cov().matrix()));
    graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
      camera_key, marker1_key,
      t_camera_marker1.tf().to<gtsam::Pose3>(),
      gtsam::noiseModel::Gaussian::Covariance(t_camera_marker1.cov().matrix()));

    // Set the initial values
    auto t_marker0_camera_i = t_camera_marker0.tf().inverse().to<gtsam::Pose3>();
    auto t_camera_marker1_i = t_camera_marker1.tf().to<gtsam::Pose3>();
    initial.insert(marker0_key, gtsam::Pose3());
    initial.insert(camera_key, t_marker0_camera_i);
    initial.insert(marker1_key, t_marker0_camera_i * t_camera_marker1_i);

    // Optimize
    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

    gtsam::Marginals marginals(graph, result);
    auto m1_f_m0_calc = result.at<gtsam::Pose3>(marker1_key);
    gtsam::Pose3::Jacobian m1_f_m0_calc_covariance = marginals.marginalCovariance(marker1_key);

    return fvlam::Transform3WithCovariance(
      fvlam::Transform3::from(m1_f_m0_calc),
      m1_f_m0_calc_covariance);
  }

  int inter_marker_rotation()
  {
    // First set up the pose of the markers and camera.
    fvlam::Translate3 m0t{0, 0, 4};
    fvlam::Translate3 m1t{2, 3, 2};
    fvlam::Translate3 c0t{1, 0, 0};
    fvlam::Rotate3 m0r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 0);
    fvlam::Rotate3 m1r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 90 * M_PI / 180.);
    fvlam::Rotate3 c0r = fvlam::Rotate3::RzRyRx(45 * M_PI / 180., 0, 0);
    fvlam::Transform3 m0_f_w{m0r, m0t};
    fvlam::Transform3 m1_f_w{m1r, m1t};
    fvlam::Transform3 c0_f_w{c0r, c0t};

    auto cov_value = 0.1;
    fvlam::Transform3 t_c0_m0 = c0_f_w.inverse() * m0_f_w;
    fvlam::Transform3 t_c0_m1 = c0_f_w.inverse() * m1_f_w;
    fvlam::Transform3::CovarianceMatrix t_c0_m0_cov(fvlam::Transform3::CovarianceMatrix::Identity() * cov_value);
    fvlam::Transform3::CovarianceMatrix t_c0_m1_cov(fvlam::Transform3::CovarianceMatrix::Identity() * cov_value);
    fvlam::Transform3WithCovariance t_c0_m0_with_cov(t_c0_m0, t_c0_m0_cov);
    fvlam::Transform3WithCovariance t_c0_m1_with_cov(t_c0_m1, t_c0_m1_cov);

    std::cout << "t_c0_m0_with_cov    " << std::endl << t_c0_m0_with_cov.to_string() << std::endl;
    std::cout << "t_c0_m1_with_cov    " << std::endl << t_c0_m1_with_cov.to_string() << std::endl;

    auto t_m0_m1_with_cov = calc_t_marker0_marker1(t_c0_m0_with_cov, t_c0_m1_with_cov);

    std::cout << "t_m0_m1_with_cov    " << std::endl << t_m0_m1_with_cov.to_string() << std::endl;
    std::cout << "t_m0_m1_truth       " << std::endl << (t_c0_m0.inverse() * t_c0_m1).to_string() << std::endl;

    return 0;
  }

  fvlam::CameraInfo camera_info_from(const cv::FileNode &ci_node)
  {
    std::uint32_t width = int(ci_node["width"]);
    std::uint32_t height = int(ci_node["height"]);

    fvlam::CameraInfo::CameraMatrix camera_matrix;
    auto k_node = ci_node["K"];
    for (int r = 0; r < fvlam::CameraInfo::CameraMatrix::MaxRowsAtCompileTime; r += 1)
      for (int c = 0; c < fvlam::CameraInfo::CameraMatrix::MaxColsAtCompileTime; c += 1) {
        camera_matrix(r, c) = k_node[r * fvlam::CameraInfo::CameraMatrix::MaxColsAtCompileTime + c];
      }

    fvlam::CameraInfo::DistCoeffs dist_coeffs;
    auto d_node = ci_node["D"];
    for (int r = 0; r < fvlam::CameraInfo::DistCoeffs::MaxSizeAtCompileTime; r += 1) {
      dist_coeffs(r) = d_node[r];
    }

    return fvlam::CameraInfo{width, height, camera_matrix, dist_coeffs};
  }

  fvlam::Observation observation_from(std::uint64_t id, const cv::FileNode &camera_f_images_node)
  {
    fvlam::Observation::Array cfi;
    for (std::size_t c = 0; c < fvlam::Observation::ArraySize; c += 1) {
      cfi[c] = fvlam::Translate2{camera_f_images_node[c][0], camera_f_images_node[c][1]};
    }
    return fvlam::Observation{id, cfi};
  }

  fvlam::Transform3 transform3_from(const cv::FileNode &marker_f_image_node)
  {
    // NOTE: fvlam::Transform3 and fiducial_vlam::Transform have variables in their mu vector
    // organized differently.
    fvlam::Transform3::MuVector mu;
    for (int r = 0; r < fvlam::Transform3::MuVector::MaxSizeAtCompileTime; r += 1)
      mu(r) = marker_f_image_node[r];

    return fvlam::Transform3{fvlam::Rotate3::RzRyRx(mu(3), mu(4), mu(5)),
                             fvlam::Translate3{mu(0), mu(1), mu(2)}};
  }

  std::vector<ImageMeasurement> load_image_measurements_from_file(const std::string &file_name)
  {
    std::vector<ImageMeasurement> image_measurements{};

    cv::FileStorage fs(file_name, cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    do {
      if (!fs.isOpened()) {
        std::cout << "Not opened." << std::endl;
        break;
      }
      auto root = fs.root();
      if (!root.isMap()) {
        std::cout << "root not map." << std::endl;
        break;
      }
      auto measurements_node = root["measurements"];
      if (!measurements_node.isSeq()) {
        std::cout << "no measurements sequence." << std::endl;
        break;
      }

      for (auto measurement_node = measurements_node.begin();
           measurement_node != measurements_node.end(); ++measurement_node) {

        // Get stamp
        std::uint64_t stamp = int((*measurement_node)["stamp"]);

        // Get CameraInfo
        auto camera_info_node = (*measurement_node)["camera_info"];
        auto camera_info = camera_info_from(camera_info_node);

        // Get MarkerMeasurements
        auto observations_node = (*measurement_node)["observations"];
        std::vector<MarkerMeasurement> marker_measurements;
        for (auto observation_node = observations_node.begin();
             observation_node != observations_node.end(); ++observation_node) {

          // Get id, observations, and t_camera_marker
          std::uint64_t id = int((*observation_node)["id"]);
          auto observation = observation_from(id, (*observation_node)["corners_f_image"]);
          auto t_camera_marker = transform3_from((*observation_node)["marker_f_camera"]);

          MarkerMeasurement marker_measurement{id, observation, t_camera_marker};
          marker_measurements.emplace_back(marker_measurement);
        }

        // Sort the marker_measurements by marker id
        std::sort(marker_measurements.begin(), marker_measurements.end(),
                  [](const MarkerMeasurement &a, const MarkerMeasurement &b) -> bool
                  {
                    return a.id() < b.id();
                  });

        ImageMeasurement image_measurement{stamp, camera_info, marker_measurements};
        image_measurements.emplace_back(image_measurement);
      }

    } while (false);

    return image_measurements;
  }

  int inter_marker_rotation_from_file()
  {
    auto image_measurements = load_image_measurements_from_file("../src/data/observations_sequence.json");

//    for (auto &image_measurement : image_measurements)
//      for (auto &measurement: image_measurement.measurements())
//        std::cout << image_measurement.stamp()
//                  << " " << measurement.id()
//                  << " " << measurement.marker_f_camera().to_string()
//                  << std::endl;

    auto cov = fvlam::Transform3::CovarianceMatrix(fvlam::Transform3::CovarianceMatrix::Identity() * 0.1);
    for (auto &image_measurement : image_measurements) {
      auto &measurements = image_measurement.measurements();
      for (std::size_t m0 = 0; m0 < measurements.size(); m0 += 1)
        for (std::size_t m1 = m0 + 1; m1 < measurements.size(); m1 += 1) {
          auto measurement0 = measurements[m0];
          auto measurement1 = measurements[m1];
          auto t_marker0_marker1 = calc_t_marker0_marker1(
            fvlam::Transform3WithCovariance(measurement0.marker_f_camera(), cov),
            fvlam::Transform3WithCovariance(measurement1.marker_f_camera(), cov));

          if (measurement0.id() == 0 && measurement1.id() == 3) {
            std::cout << image_measurement.stamp()
                      << " " << measurement0.id() << " " << measurement1.id()
                      << " " << t_marker0_marker1.tf().to_string()
                      << std::endl;
          }
        }
    }

    return 0;
  }

  int test_transform_from_observation()
  {

    return 0;
  }

  int build_marker_map_from_file()
  {
    auto image_measurements = load_image_measurements_from_file("../src/data/observations_sequence.json");

    auto map_initial = std::make_unique<fvlam::MarkerMap>(0.21);
    map_initial->add_marker(fvlam::Marker{
      0, fvlam::Transform3WithCovariance{}, true});

    auto solve_tmm_context = fvlam::SolveTmmContextCvSolvePnp{true};
    auto solve_tmm_factory = fvlam::make_solve_tmm_factory(solve_tmm_context,
                                                           map_initial->marker_length());

    fvlam::LoggerCout logger{fvlam::Logger::Levels::level_info};
    auto tmm_context = fvlam::BuildMarkerMapTmmContext(solve_tmm_factory,
                                                       true,
                                                       fvlam::BuildMarkerMapTmmContext::NoiseStrategy::minimum,
                                                       0.1, 0.3);
    auto map_builder = make_build_marker_map(tmm_context, logger, *map_initial);

    for (auto &image_measurement : image_measurements) {
      auto &measurements = image_measurement.measurements();
      auto observations{fvlam::Observations::from(measurements)};
      map_builder->process(observations, image_measurement.camera_info());
    }

    auto built_map = map_builder->build();
    logger.info() << "built map:" << std::endl
                  << built_map->to_string() << std::endl;
    auto error = fvlam::BuildMarkerMapTmmContext::BuildError::from(*map_builder, *built_map);
    logger.info() << "remeasure error - r:" << error.r_remeasure_error_ << " t:" << error.t_remeasure_error_;

    return 0;
  }
}
