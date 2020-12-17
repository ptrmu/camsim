
#include <memory>
#include <fvlam/camera_info.hpp>

#include "fvlam/build_marker_map_interface.hpp"

namespace fvlam
{

  class BuildMarkerMapShonan : public BuildMarkerMapInterface
  {
  public:
    BuildMarkerMapShonan() = default;

    void process_image_observations(const MarkerObservations &marker_observations,
                                    const CameraInfo &camera_info) override
    {

    }

    std::string build_marker_map(MarkerMap &map) override
    {
      return std::string{};
    }

    std::string reset(std::string &cmd) override
    {
      return std::string{};
    }
  };

  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map_shonan()
  {
    return std::make_unique<BuildMarkerMapShonan>();
  }

}
