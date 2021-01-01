
#include "pfp_run.hpp"

#include "../model.hpp"
#include "../pose_with_covariance.hpp"

namespace camsim
{
  typedef std::reference_wrapper<const MarkerModel> MarkerModelRef;

  class TestGspnp
  {
    Model &model_;

    double calc_tz(gtsam::Point2 corner_f_image0,
                   gtsam::Point2 corner_f_image1,
                   gtsam::Point3 corner_f_world0,
                   gtsam::Point3 corner_f_world1)
    {
      // t/f = d_world / d_image
      auto d_image = (corner_f_image1 - corner_f_image0).norm();
      auto d_world = (corner_f_world1 - corner_f_world0).norm();

      return model_.cameras_.calibration_.fx() * d_world / d_image;
    }

    void calc_gspnp(const CameraModel &camera,
                    const MarkerModel &marker,
                    const std::vector<gtsam::Point2> corners_f_image)
    {
      auto camera_f_marker = camera.camera_f_world_.inverse() * marker.marker_f_world_;
      std::cout << PoseWithCovariance::to_str(camera_f_marker) << std::endl;
      std::cout << calc_tz(corners_f_image[0],
                           corners_f_image[1],
                           model_.markers_.corners_f_marker_[0],
                           model_.markers_.corners_f_marker_[1]) << " "
                << calc_tz(corners_f_image[0],
                           corners_f_image[2],
                           model_.markers_.corners_f_marker_[0],
                           model_.markers_.corners_f_marker_[2]) << " "
                << calc_tz(corners_f_image[0],
                           corners_f_image[3],
                           model_.markers_.corners_f_marker_[0],
                           model_.markers_.corners_f_marker_[3]) << std::endl;
    }

  public:
    TestGspnp(Model &model) :
      model_{model}
    {}

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      for (auto &marker_ref : marker_refs) {
        auto &marker = marker_ref.get();
        auto &corners_f_image = model_.corners_f_images_[camera.index()][marker.index()];
        calc_gspnp(camera, marker, corners_f_image.corners_f_image_);
      }
    }
  };

  static void test_one_camera(const CameraModel &camera,
                              const std::vector<MarkerModelRef> &marker_refs)
  {
  }

  int pfp_gspnp()
  {
    int n_markers = 8;
    int n_cameras = 8;
    (void) n_markers;
    (void) n_cameras;

    ModelConfig model_config{[]() -> std::vector<gtsam::Pose3>
                             {
                               return std::vector<gtsam::Pose3>{gtsam::Pose3{gtsam::Rot3::RzRyRx(M_PI / 20, 0., 0.),
                                                                             gtsam::Point3::Zero()}};
                             },
                             PoseGens::CubeAlongZFacingOrigin{3, 2., 2.},
                             camsim::CameraTypes::simulation,
                             0.1775

    };
//    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingOrigin{n_markers, 2.},
//                            PoseGens::SpinAboutZAtOriginFacingOut{n_cameras},
//                            camsim::CameraTypes::simulation,
//                            0.1775}};

//    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingAlongZ{n_markers, 2., 2., false},
//                            PoseGens::CircleInXYPlaneFacingAlongZ{n_cameras, 2., 0., true},
//                            camsim::CameraTypes::simulation,
//                            0.1775}};

    Model model{model_config};

    TestGspnp test_gspnp{model};

    // Loop over all the cameras
    for (auto &camera : model.cameras_.cameras_) {
      std::vector<MarkerModelRef> marker_refs{};

      // Figure out which markers are visible from this camera
      for (auto &marker : model.markers_.markers_) {
        if (!model.corners_f_images_[camera.index()][marker.index()].corners_f_image_.empty()) {
          marker_refs.emplace_back(MarkerModelRef{marker});
        }
      }

      // Let the solver work on these measurements.
      test_gspnp(camera, marker_refs);
    }

    return 0;
  }
}
