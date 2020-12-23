
#include <memory>
#include <fvlam/camera_info.hpp>

#include "fvlam/build_marker_map_interface.hpp"
#include "fvlam/marker_map.hpp"

namespace fvlam
{

  class BuildMarkerMapShonan : public BuildMarkerMapInterface
  {
    const BuildMarkerMapShonanContext context_;
    std::unique_ptr<MarkerMap> map_initial_;

    MarkerObservations marker_observations_{};
    CameraInfo camera_info_{};

  public:
    BuildMarkerMapShonan() = delete;

    BuildMarkerMapShonan(const BuildMarkerMapShonanContext &context,
                         std::unique_ptr<MarkerMap> map) :
      context_{context},
      map_initial_{std::move(map)}
    {}

    void process_image_observations(const MarkerObservations &marker_observations,
                                    const CameraInfo &camera_info) override
    {
      marker_observations_ = marker_observations;
    }

    std::unique_ptr<MarkerMap> build_marker_map() override
    {
      return std::make_unique<fvlam::MarkerMap>(map_initial_->marker_length());
    }

    std::string reset(std::string &cmd) override
    {
      return std::string{};
    }
  };

  template<>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map<BuildMarkerMapShonanContext>(
    const BuildMarkerMapShonanContext &context,
    std::unique_ptr<MarkerMap> map_initial)
  {
    return std::make_unique<BuildMarkerMapShonan>(context, std::move(map_initial));
  }
}
