

#include "fvlam/camera_info.hpp"
#include "fvlam/logger.hpp"
#include "fvlam/marker.hpp"
#include "fvlam/observation.hpp"
#include "fvlam/observations_bundle.hpp"
#include "fvlam/transform3_with_covariance.hpp"
#include "opencv2/core/persistence.hpp"

namespace fvlam
{

// ==============================================================================
// MarkerMap save methods
// ==============================================================================

  template<>
  void Translate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "[";
    other << t_(0);
    other << t_(1);
    other << t_(2);
    other << "]";
  }

  template<>
  void Rotate3::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    auto r = xyz();
    other << "[";
    other << r(2);
    other << r(1);
    other << r(0);
    other << "]";
  }

  template<>
  void CameraInfo::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
//
//    other << "id" << int(id_);
//    other << "f" << (is_fixed_ ? 1 : 0);
//    other << "xyz";
//    t_world_marker_.tf().t().to(other);
//    other << "rpy";
//    t_world_marker_.tf().r().to(other);
//
    other << "}";
  }

  template<>
  void Marker::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";

    other << "id" << int(id_);
    other << "f" << (is_fixed_ ? 1 : 0);
    other << "xyz";
    t_world_marker_.tf().t().to(other);
    other << "rpy";
    t_world_marker_.tf().r().to(other);

    other << "}";
  }

  template<>
  void MarkerMap::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "marker_length" << marker_length();
    other << "markers" << "[";

    for (auto &marker : markers_) {
      marker.second.to(other);
    }

    other << "]";
    other << "}";
  }

  template<>
  void Observation::to<cv::FileStorage>(cv::FileStorage &other) const
  {
//    other << "{";
//    other << "stamp" << stamp_;
//    other << "observations"<< "[";
//
//    for (auto &observation : observations_) {
//      observation.to(other);
//    }
//
//    other << "]";
//    other << "}";
  }

  template<>
  void Observations::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
//    other << "stamp" << stamp_;
    other << "observations" << "[";

    for (auto &observation : observations_) {
      observation.to(other);
    }

    other << "]";
    other << "}";
  }

  template<>
  void ObservationsBundle::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "camera_info";
    camera_info_.to(other);
    other << "observations";
    observations_.to(other);
    other << "}";
  }

  template<>
  void ObservationsBundles::to<cv::FileStorage>(cv::FileStorage &other) const
  {
    other << "{";
    other << "map";
    map_.to(other);
    other << "bundles" << "[";

    for (auto &bundle : bundles_) {
      bundle.to(other);
    }

    other << "]";
    other << "}";
  }


// ==============================================================================
// MarkerMap load methods
// ==============================================================================

  struct FileStorageContext
  {
    Logger &logger_;
    bool success_{true};

    class Node
    {
      FileStorageContext &cxt_;
      cv::FileNode node_;

    public:
      Node(FileStorageContext &cxt, cv::FileNode node)
        : cxt_{cxt}, node_{node}
      {}

      Node make(cv::FileNode file_node)
      {
        return Node{cxt_, file_node};
      }

      cv::FileNode operator()()
      { return node_; }

      FileStorageContext &cxt()
      { return cxt_; }
    };

    FileStorageContext(Logger &logger) :
      logger_{logger}
    {}

    Node make(cv::FileNode file_node)
    {
      return Node{*this, file_node};
    }

    void set_failed()
    {
      success_ = false;
    }

    bool success()
    {
      return success_;
    }
  };

  template<>
  Translate3 Translate3::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    double x = other()[0];
    double y = other()[1];
    double z = other()[2];
    return Translate3{x, y, z};
  }

  template<>
  Rotate3 Rotate3::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    double rx = other()[0];
    double ry = other()[1];
    double rz = other()[2];
    return Rotate3::RzRyRx(rx, ry, rz);
  }

  template<>
  Marker Marker::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    int id = other()["id"];
    int fixed = other()["f"];

    auto t_node = other()["xyz"];
    auto t_context = other.make(t_node);
    auto t = Translate3::from(t_context);

    auto r_node = other()["rpy"];
    auto r_context = other.make(r_node);
    auto r = Rotate3::from(r_context);

    return Marker{std::uint64_t(id), Transform3WithCovariance{Transform3{r, t}}, fixed != 0};
  }

  template<>
  MarkerMap MarkerMap::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
    double marker_length = other()["marker_length"];
    MarkerMap map{marker_length};

    auto markers_node = other()["markers"];
    for (auto it = markers_node.begin(); it != markers_node.end(); ++it) {
      auto marker_context = other.make(*it);
      auto marker = Marker::from(marker_context);
      map.add_marker(marker);
    }

    return map;
  }
//
//  template<>
//  MarkerMap MarkerMap::from<FileStorageContext::Node>(FileStorageContext::Node &other)
//  {
//    double marker_length = other()["marker_length"];
//    MarkerMap map{marker_length};
//
//    auto markers_node = other()["markers"];
//    for (auto it = markers_node.begin(); it != markers_node.end(); ++it) {
//      auto marker_context = other.make(*it);
//      auto marker = Marker::from(marker_context);
//      map.add_marker(marker);
//    }
//
//    return map;
//  }

  template<>
  ObservationsBundles ObservationsBundles::from<FileStorageContext::Node>(FileStorageContext::Node &other)
  {
//    double marker_length = other()["marker_length"];
//    MarkerMap map{marker_length};
//
//    auto markers_node = other()["markers"];
//    for (auto it = markers_node.begin(); it != markers_node.end(); ++it) {
//      auto marker_context = other.make(*it);
//      auto marker = Marker::from(marker_context);
//      map.add_marker(marker);
//    }
//
//    return map;
    return ObservationsBundles{MarkerMap{0.0}};
  }

// ==============================================================================
// MarkerMap class
// ==============================================================================

  void MarkerMap::save(const std::string filename, Logger &logger) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not create MarkerMap file :" << filename;
      return;
    }

    to(fs);
  }

  MarkerMap MarkerMap::load(const std::string filename, Logger &logger)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not open MarkerMap file :" << filename;
      return MarkerMap{0.0};
    }

    FileStorageContext context{logger};
    auto root_node = context.make(fs.root());

    auto map = MarkerMap::from(root_node);
    return context.success() ? map : MarkerMap{0.0};
  }


// ==============================================================================
// ObservationsBundles class
// ==============================================================================

  void ObservationsBundles::save(const std::string filename, Logger &logger) const
  {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not create MarkerMap file :" << filename;
      return;
    }

    to(fs);
  }

  ObservationsBundles ObservationsBundles::load(const std::string filename, Logger &logger)
  {
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if (!fs.isOpened()) {
      logger.error() << "Could not open ObservationsBundles file :" << filename;
      return MarkerMap{0.0};
    }

    FileStorageContext context{logger};
    auto root_node = context.make(fs.root());

    auto map = ObservationsBundles::from(root_node);
    return context.success() ? map : MarkerMap{0.0};
  }
}
