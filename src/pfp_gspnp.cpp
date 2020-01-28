
#include "pfp_run.hpp"

#include "model.hpp"

namespace camsim
{
  typedef std::reference_wrapper<const MarkerModel> MarkerModelRef;

  class TestGspnp
  {
    Model &model_;

  public:
    TestGspnp(Model &model) :
      model_{model}
    {}

    void operator()(const CameraModel &camera,
                    const std::vector<MarkerModelRef> &marker_refs)
    {
      for (auto &marker : marker_refs) {

        
      }
    }
  };

  static void test_one_camera(const CameraModel &camera,
                              const std::vector<MarkerModelRef> &marker_refs)
  {
  }

  void pfp_gspnp()
  {
    int n_markers = 8;
    int n_cameras = 32;

    Model model{ModelConfig{PoseGens::CircleInXYPlaneFacingAlongZ{n_markers, 2., 2., false},
                            PoseGens::CircleInXYPlaneFacingAlongZ{n_cameras, 2., 0., true},
                            camsim::CameraTypes::simulation,
                            0.1775}};

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
      if (marker_refs.size() > 1) {
        test_gspnp(camera, marker_refs);
      }
    }
  }
}
