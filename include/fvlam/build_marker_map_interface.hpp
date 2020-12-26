#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  class CameraInfo; //
  class MarkerMap; //
  class MarkerObservation; //
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
// SolveTMarker0Marker1Interface class
// ==============================================================================

// An interface used to calculate t_marker0_marker1. There are several ways to do
// this calculation and this interface is common to them all. This interface is
// meant to accumulate the results from several observations and estimate statistics
// of the result. One instance of this interface is for one measurement. Multiple
// instances are needed for multiple instances.
  class SolveTMarker0Marker1Interface
  {
  public:
    virtual ~SolveTMarker0Marker1Interface() = default;

    // Take the location of markers in one image and add them to the marker map
    // building algorithm.
    virtual void accumulate(const MarkerObservation &observation0,
                            const MarkerObservation &observation1) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual Transform3WithCovariance t_marker0_marker1() = 0;
  };

  using SolveTMarker0Marker1Factory = std::function<std::unique_ptr<SolveTMarker0Marker1Interface>()>;

  template<class TContext>
  SolveTMarker0Marker1Factory make_solve_t_marker0_marker1_factory(const TContext &context,
                                                                   const CameraInfo &camera_info,
                                                                   double marker_length);

  // Solve t_marker0_marker1 using OpenCV's SolvePnp and then average the results
  // on the SE3 manifold or the se3 tangent space
  struct SolveTmmCvSolvePnp
  {
    bool average_on_space_not_manifold{true};
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
//      if (n_ == 0) {
//        first_sample_ = mu;
//        first_cov_ = cov;
//      }
      MUVECTOR mu_centered = mu - first_sample_;
      mu_sum_ += mu_centered;
      mu_mu_sum_ += mu_centered * mu_centered.transpose();
      n_ += 1;
    }

    MUVECTOR mean() const
    {
      if (n_ == 0) {
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

  class EstimateTransform3MeanAndCovariance
  {
    EstimateMeanAndCovariance<Transform3::MuVector> emac_;
    bool first_{false};
    fvlam::Transform3 offset_{};
    fvlam::Transform3 offset_inverse_{};

  public:
    explicit EstimateTransform3MeanAndCovariance(std::uint64_t id) :
      emac_{id}
    {}

    void accumulate(const Transform3 &tr)
    {
      if (first_) {
        offset_ = tr;
        offset_inverse_ = tr.inverse();
        first_ = false;
      }
      auto tr_adj = tr * offset_inverse_;
      auto log_tr = Transform3::Logmap(tr_adj);
      emac_.accumulate(log_tr, Transform3::CovarianceMatrix::Zero());
//        emac_.accumulate(tr.mu(), Transform3::CovarianceMatrix::Zero());
    }

    Transform3 mean() const
    {
      return Transform3::Expmap(emac_.mean()) * offset_;
//      return Transform3{emac_.mean()};
    }

    Transform3::CovarianceMatrix cov() const
    {
      return emac_.cov();
    }
  };
}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
