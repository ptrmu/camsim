
#include "pfm_run.hpp"
#include "pose_with_covariance.hpp"
#include <random>

namespace camsim
{

  static int constexpr num_samples = 4096;

  static std::vector<std::vector<gtsam::Point2>> generate_normal_corners_f_images(
    const std::vector<gtsam::Point2> &corners_f_image,
    const gtsam::SharedNoiseModel &measurement_noise)
  {
    // Allocate normal random number generators
//    std::random_device rd{};
//    std::mt19937 gen{rd()};
    std::mt19937 gen{17};

    // Create a distribution for each point coordinate
    std::vector<std::pair<std::normal_distribution<>, std::normal_distribution<>>> dists{};
    for (auto &corner_f_image : corners_f_image) {
      dists.emplace_back(std::pair<std::normal_distribution<>, std::normal_distribution<>>{
        std::normal_distribution<>{corner_f_image.x(), measurement_noise->sigmas()(0)},
        std::normal_distribution<>{corner_f_image.y(), measurement_noise->sigmas()(1)}
      });
    }

    // Generate a lot of samples
    std::vector<std::vector<gtsam::Point2>> image_points_samples(num_samples);
    for (auto &image_points_sample : image_points_samples) {
      std::vector<gtsam::Point2> sample{};
      for (auto &dist : dists) {
        sample.emplace_back(gtsam::Point2{dist.first(gen), dist.second(gen)});
      }
      image_points_sample = std::move(sample);
    }

    return image_points_samples;
  }

  static std::vector<gtsam::Pose3> many_opencv_resection(const gtsam::Cal3_S2 &camera_calibration,
                                                         std::vector<std::vector<gtsam::Point2>> &corners_f_images,
                                                         const std::vector<gtsam::Point3> &corners_f_world,
                                                         const gtsam::Pose3 &camera_f_world_initial,
                                                         const gtsam::SharedNoiseModel &measurement_noise)
  {
    std::vector<gtsam::Pose3> camera_f_worlds{};

    for (auto &corners_f_image : corners_f_images) {
      gtsam::Pose3 camera_f_world;
      gtsam::Matrix6 camera_f_world_covariance;
      pfm_resection_gtsam(camera_calibration,
                          corners_f_image,
                          corners_f_world,
                          camera_f_world_initial,
                          measurement_noise,
                          camera_f_world, camera_f_world_covariance);

      camera_f_worlds.emplace_back(camera_f_world);
//      std::cout << PoseWithCovariance::to_str(camera_f_world_) << std::endl;
    }

    return camera_f_worlds;
  }

  static void mean_camera_f_world(const gtsam::Pose3 &camera_f_world_initial,
                                  std::vector<gtsam::Pose3> camera_f_worlds,
                                  gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance)
  {
    camera_f_world_covariance = gtsam::Z_6x6;

    if (camera_f_worlds.empty()) {
      return;
    }
    if (camera_f_worlds.size() < 2) {
      camera_f_world = camera_f_worlds[0];
    }

    // The mean calculation. Not sure if this is a valid technique.
    gtsam::Vector6 logmap_sum{};
    for (auto &cam_f_world: camera_f_worlds) {
      logmap_sum += camera_f_world_initial.logmap(cam_f_world);
    }
    logmap_sum /= camera_f_worlds.size();
    camera_f_world = camera_f_world_initial.expmap(logmap_sum);

    // Calculate variance
    gtsam::Vector6 mean{(gtsam::Vector(6) << camera_f_world.rotation().xyz(), camera_f_world.translation()).finished()};
    gtsam::Matrix6 cov = gtsam::Z_6x6;
    for (auto &cam_f_world: camera_f_worlds) {
      gtsam::Vector6 res{(gtsam::Vector(6) << cam_f_world.rotation().xyz(), cam_f_world.translation()).finished()};
      res -= mean;
      res(0) = res(0) < -M_PI ? res(0) + 2 * M_PI : (res(0) > M_PI ? res(0) - 2 * M_PI : res(0));
      res(1) = res(1) < -M_PI ? res(1) + 2 * M_PI : (res(1) > M_PI ? res(1) - 2 * M_PI : res(1));
      res(2) = res(2) < -M_PI ? res(2) + 2 * M_PI : (res(2) > M_PI ? res(2) - 2 * M_PI : res(2));
      auto cross_res = res * res.transpose();
      cov += cross_res;
    }

    cov /= camera_f_worlds.size();

    camera_f_world_covariance = cov;
  }

  void pfm_resection_monte_carlo(const gtsam::Cal3_S2 &camera_calibration,
                                 const std::vector<gtsam::Point2> &corners_f_image,
                                 const std::vector<gtsam::Point3> &corners_f_world,
                                 const gtsam::Pose3 &camera_f_world_initial,
                                 const gtsam::SharedNoiseModel &measurement_noise,
                                 gtsam::Pose3 &camera_f_world, gtsam::Matrix6 &camera_f_world_covariance)
  {
    // Generate lots of corners_f_image
    auto corners_f_images = generate_normal_corners_f_images(corners_f_image, measurement_noise);

    // Generate a pose for each of these possible sets of corners
    auto camera_f_worlds = many_opencv_resection(camera_calibration,
                                                 corners_f_images,
                                                 corners_f_world,
                                                 camera_f_world_initial,
                                                 measurement_noise);

    // Find the mean and covariance of these poses
    mean_camera_f_world(camera_f_world_initial,
                        camera_f_worlds,
                        camera_f_world, camera_f_world_covariance);
  }
}
