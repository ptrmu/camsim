#pragma once

#include "camera_info.hpp"
#include "marker.hpp"
#include "observation.hpp"

namespace fvlam
{
// ==============================================================================
// ObservationsBundle class
// ==============================================================================

  class ObservationsBundle
  {
    CameraInfo camera_info_;
    Observations observations_;

  public:
    ObservationsBundle(CameraInfo camera_info, Observations observations) :
      camera_info_{std::move(camera_info)}, observations_{std::move(observations)}
    {}

    template<class T>
    static ObservationsBundle from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;
  };

  class ObservationsBundles
  {
    MarkerMap map_;
    std::vector<ObservationsBundle> bundles_;

  public:
    ObservationsBundles(MarkerMap map) :
      map_{std::move(map)}
    {}

    void add_bundle(const ObservationsBundle &bundle)
    {
      bundles_.emplace_back(bundle);
    }

    template<class T>
    static ObservationsBundles from(T &other);

    template<class T>
    T to() const;

    template<class T>
    void to(T &other) const;

    void save(const std::string filename, Logger &logger) const; //
    static ObservationsBundles load(const std::string filename, Logger &logger); //
  };
}