
#include "calibration_model.hpp"

namespace camsim
{
//  static const std::vector<BoardModel> gen_boards(const ModelConfig &cfg, const )
//  {
//    if (cfg.markers_configuration_ != MarkersConfigurations::generator) {
//      return std::vector<BoardModel>{};
//    }
//
//    auto boards_f_world = cfg.marker_pose_generator_();
//
//    std::vector<BoardModel> boards{};
//    for (std::size_t idx = 0; idx < boards_f_world.size(); idx += 1) {
//      auto &board_f_world = boards_f_world[idx];
//
//      std::vector<gtsam::Point3> corners_f_world{};
//      for (auto &corner_f_marker : corners_f_marker) {
//        corners_f_world.emplace_back(marker_f_world * corner_f_marker);
//      }
//
//      auto marker_key{MarkerModel::marker_key(idx)};
//      markers.emplace_back(MarkerModel{MarkerModel::marker_key(idx), marker_f_world,
//                                       CornersModel(idx, marker_f_world, corners_f_marker),
//                                       std::move(corners_f_world)});
//    }
//
//    return markers;
//  }
//
//  static const std::vector<PointFBoard> gen_junctions_f_board(const CheckerboardsModel::Config &ch_cfg)
//  {
//
//  }
}
