#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  class CameraInfo; //
  class MarkerMap; //
  class MarkerObservations; //

// ==============================================================================
// BuildMarkerMapInterface class
// ==============================================================================

// An interface used to build maps of markers. This is a common interface to
// several modules that use different techniques to build maps.
  class BuildMarkerMapInterface
  {
  public:
    virtual ~BuildMarkerMapInterface() = default;

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    virtual void process(const MarkerObservations &marker_observations,
                         const CameraInfo &camera_info) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual std::unique_ptr<MarkerMap> build() = 0;

    // Re-initialize the map_builder. (i.e. through out any data accumulated so far)
    virtual std::string reset(std::string &cmd) = 0;
  };

  template<class TContext>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map(const TContext &context,
                                                                 const MarkerMap &map_initial);

// ==============================================================================
// BuildMarkerMapShonanContext class
// ==============================================================================

  class BuildMarkerMapShonanContext
  {
  public:
    const int flags_;

    explicit BuildMarkerMapShonanContext(int flags) :
      flags_{flags}
    {}

    template<class T>
    static Transform3 from(const T &other);
  };

// ==============================================================================
// EstimateMeanAndCovariance class
// ==============================================================================

  template<class MUVECTOR>
  class EstimateMeanAndCovariance
  {
    using CovarianceMatrix = Eigen::Matrix<double, MUVECTOR::MaxSizeAtCompileTime, MUVECTOR::MaxSizeAtCompileTime>;
    std::uint64_t id_;
    MUVECTOR first_sample_{MUVECTOR::Zero()};
    CovarianceMatrix first_cov_{CovarianceMatrix::Zero()};
    MUVECTOR mu_sum_{MUVECTOR::Zero()};
    CovarianceMatrix mu_mu_sum_{CovarianceMatrix::Zero()};
    std::uint64_t n_{0};

  public:
    explicit EstimateMeanAndCovariance(std::uint64_t id) :
      id_{id}
    {}

    void accumulate(const MUVECTOR &mu, const CovarianceMatrix cov)
    {
      if (n_ == 0) {
        first_sample_ = mu;
        first_cov_ = cov;
      }
      MUVECTOR mu_centered = mu - first_sample_;
      mu_sum_ += mu_centered;
      mu_mu_sum_ += mu_centered * mu_centered.transpose();
      n_ += 1;
    }

    MUVECTOR mean() const
    {
      if (n_ <= 0) {
        return mu_sum_ + first_sample_;
      }
      return mu_sum_ / n_ + first_sample_;
    }

    CovarianceMatrix cov() const
    {
      if (n_ <= 1) {
        return first_cov_;
      }
      return (mu_mu_sum_ - (mu_sum_ * mu_sum_.transpose()) / n_) / n_;
    }
  };
}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
