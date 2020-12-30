#ifndef FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
#define FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP

#include <memory>

#include "fvlam/transform3_with_covariance.hpp"

namespace fvlam
{

  class CameraInfo; //
  class MarkerMap; //
  class Observation; //
  class Observations; //

// ==============================================================================
// Logger class
// ==============================================================================

  class Logger
  {
  public:
    enum Levels
    {
      level_debug = 0,
      level_info,
      level_error
    };

    class StreamProxy
    {
      bool output_;
      std::basic_ostream<char> &ostream_;

    public:
      StreamProxy(std::basic_ostream<char> &ostream, bool output) :
        output_{output}, ostream_{ostream}
      {}

      template<typename V>
      StreamProxy &operator<<(V const &value)
      {
        if (output_) {
          ostream_ << value;
        }
        return *this;
      }

      StreamProxy &operator<<(std::basic_ostream<char> &(*func)(std::basic_ostream<char> &))
      {
        if (output_) {
          func(ostream_);
        }
        return *this;
      }
    };

  private:
    std::basic_ostream<char> &ostream_;
    Levels output_level_;

  public:
    Logger(std::basic_ostream<char> &ostream, Levels output_level) :
      ostream_{ostream}, output_level_{output_level}
    {}

    // These methods return a new StreamProxy object that will stream or not.
    // This temporary StreamProxy object is held alive while the subsequent << operators
    // pass a reference to it along. The StreamProxy object is then destroyed at the end
    // of the statement. This is convenient compiler magic.
    StreamProxy debug()
    { return StreamProxy{ostream_, level_debug >= output_level_}; } //
    StreamProxy info()
    { return StreamProxy{ostream_, level_info >= output_level_}; } //
    StreamProxy error()
    { return StreamProxy{ostream_, level_error >= output_level_}; } //
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
    virtual void accumulate(const Observation &observation0,
                            const Observation &observation1,
                            const CameraInfo &camera_info) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual Transform3WithCovariance t_marker0_marker1() = 0;
  };

  using SolveTMarker0Marker1Factory = std::function<std::unique_ptr<SolveTMarker0Marker1Interface>()>;

  template<class TSolveTmmContext>
  SolveTMarker0Marker1Factory make_solve_tmm_factory(const TSolveTmmContext &solve_tmm_context,
                                                     double marker_length);

  // Solve t_marker0_marker1 using OpenCV's SolvePnp and then average the results
  // on the SE3 manifold or the se3 tangent space
  struct SolveTmmContextCvSolvePnp
  {
    bool average_on_space_not_manifold{true};
  };

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
    virtual void process(const Observations &observations,
                         const CameraInfo &camera_info) = 0;

    // Given the observations that have been added so far, create and return a marker_map.
    virtual std::unique_ptr<MarkerMap> build() = 0;

    // Calculate the average angular and linear error between the measured
    // t_marker0_marker1 and the t_marker0_marker1 for markers in the map.
    virtual std::pair<double, double> error(const MarkerMap &map) = 0;
  };

  template<class TBmmContext>
  std::unique_ptr<BuildMarkerMapInterface> make_build_marker_map(const TBmmContext &tmm_context,
                                                                 SolveTMarker0Marker1Factory solve_tmm_factory,
                                                                 const MarkerMap &map_initial);

// ==============================================================================
// BuildMarkerMapTmmContext class
// ==============================================================================

  struct BuildMarkerMapTmmContext
  {
    enum struct NoiseStrategy
    {
      estimation, // Use estimate from samples
      minimum, // Use fixed value if estimate is below fixed
      fixed, // Use fixed value
    };

    bool try_shonan_initialization_{true};
    NoiseStrategy mm_between_factor_noise_strategy_{NoiseStrategy::minimum};
    double mm_between_factor_noise_fixed_sigma_r_{0.1};
    double mm_between_factor_noise_fixed_sigma_t_{0.3};


    explicit BuildMarkerMapTmmContext(bool try_shonan_initialization,
                                      NoiseStrategy mm_between_factor_noise_strategy,
                                      double mm_between_factor_noise_fixed_sigma_r = 0.1,
                                      double mm_between_factor_noise_fixed_sigma_t = 0.3) :
      try_shonan_initialization_{try_shonan_initialization},
      mm_between_factor_noise_strategy_{mm_between_factor_noise_strategy},
      mm_between_factor_noise_fixed_sigma_r_{mm_between_factor_noise_fixed_sigma_r},
      mm_between_factor_noise_fixed_sigma_t_{mm_between_factor_noise_fixed_sigma_t}
    {}

    template<class T>
    static Transform3 from(const T &other);
  };

// ==============================================================================
// EstimateTransform3MeanAndCovariance class
// ==============================================================================

  template<class MUVECTOR>
  class EstimateMeanAndCovarianceSimple
  {
  public:
    using CovarianceMatrix = Eigen::Matrix<double, MUVECTOR::MaxSizeAtCompileTime, MUVECTOR::MaxSizeAtCompileTime>;

  protected:
    MUVECTOR mu_sum_{MUVECTOR::Zero()};
    CovarianceMatrix mu_mu_sum_{CovarianceMatrix::Zero()};
    std::uint64_t n_{0};

  public:

    void accumulate(const MUVECTOR &mu)
    {
      mu_sum_ += mu;
      mu_mu_sum_ += mu * mu.transpose();
      n_ += 1;
    }

    MUVECTOR mean() const
    {
      return (n_ == 0) ? mu_sum_ : mu_sum_ / n_;
    }

    CovarianceMatrix cov() const
    {
      return (n_ == 0) ? mu_mu_sum_ : (mu_mu_sum_ - (mu_sum_ * mu_sum_.transpose()) / n_) / n_;
    }
  };

  class EstimateTransform3MeanAndCovariance : public EstimateMeanAndCovarianceSimple<fvlam::Transform3::TangentVector>
  {
    using Base = EstimateMeanAndCovarianceSimple<fvlam::Transform3::TangentVector>;
    fvlam::Rotate3 r_adj_{};
    fvlam::Rotate3 r_adj_inverse_{};
    fvlam::Translate3 t_adj_{};
    fvlam::Translate3 t_adj_inverse_{};

  public:

    void accumulate(const Transform3 &tr)
    {
      if (Base::n_ == 0) {
        r_adj_ = tr.r();
        t_adj_ = tr.t();
        r_adj_inverse_ = r_adj_.inverse();
        t_adj_inverse_ = t_adj_ * -1;
      }
      auto tr_adj = Transform3{tr.r() * r_adj_inverse_, tr.t() + t_adj_inverse_};
      Base::accumulate(fvlam::Transform3::ChartAtOrigin::local(tr_adj));
    }

    Transform3 mean() const
    {
      auto tr_adj = fvlam::Transform3::ChartAtOrigin::retract(Base::mean());
      auto tr = Transform3{tr_adj.r() * r_adj_, tr_adj.t() + t_adj_};
      return tr;
    }
  };


  template<class MUVECTOR>
  class EstimateMeanAndCovariance : public EstimateMeanAndCovarianceSimple<MUVECTOR>
  {
    using Base = EstimateMeanAndCovarianceSimple<MUVECTOR>;
    MUVECTOR first_sample_{MUVECTOR::Zero()};

  public:

    void accumulate(const MUVECTOR &mu)
    {
      if (Base::n_ == 0) {
        first_sample_ = mu;
      }
      MUVECTOR mu_adj = mu - first_sample_;
      Base::accumulate(mu_adj);
    }

    MUVECTOR mean() const
    {
      return Base::mean() + first_sample_;
    }
  };

  template<class MUVECTOR>
  class EstimateMeanAndCovariance2PSimple
  {
  public:
    using MuVector = MUVECTOR;
    using CovarianceMatrix = Eigen::Matrix<double, MUVECTOR::MaxSizeAtCompileTime, MUVECTOR::MaxSizeAtCompileTime>;
    std::vector<MUVECTOR> mus_{};

  public:

    void accumulate(const MUVECTOR &mu)
    {
      mus_.template emplace_back(mu);
    }

    MUVECTOR mean() const
    {
      MUVECTOR mu_sum{MUVECTOR::Zero()};
      for (auto &mu : mus_) {
        mu_sum = mu_sum + mu;
      }
      auto mu_mean = mu_sum / mus_.size();
      return mu_mean;
    }

    CovarianceMatrix cov() const
    {
      CovarianceMatrix mu_mu_sum{CovarianceMatrix::Zero()};
      auto m = mean();
      for (auto &mu : mus_) {
        auto mu_deviate = mu - m;
        mu_mu_sum = mu_mu_sum + mu_deviate * mu_deviate.transpose();
      }
      CovarianceMatrix cov = mu_mu_sum / mus_.size();
      return cov;
    }
  };

#if 0

  class EstimateTransform3MeanAndCovarianceEade : public EstimateMeanAndCovariance2PSimple<fvlam::Transform3::MuVector>
  {
    using Base = EstimateMeanAndCovariance2PSimple<fvlam::Transform3::MuVector>;

    fvlam::Transform3 iterate() const
    {
      auto mean_k = Transform3{Base::mus_[0]};
      auto mean_k_inverse = mean_k.inverse();

      for (int k = 0; k < 4; k += 1) {
        Base::MuVector deviation_ln_sum{Base::MuVector::Zero()};

        for (int i = 0; i < Base::mus_.size(); i += 1) {
          auto deviation = Transform3{Base::mus_[i]} * mean_k_inverse;
          auto deviation_ln = Transform3::Logmap(deviation);
          deviation_ln_sum += deviation_ln;
        }

        mean_k = Transform3::Expmap(deviation_ln_sum / Base::mus_.size()) * mean_k;
        mean_k_inverse = mean_k.inverse();
      }
      return mean_k;
    }

  public:

    void accumulate(const Transform3 &tr)
    {
      Base::accumulate(tr.mu());
    }

    fvlam::Transform3 mean() const
    {
      return iterate();
    }

    CovarianceMatrix cov() const
    {
      return CovarianceMatrix::Zero();
    }
  };

#endif

}
#endif //FVLAM_BUILD_MARKER_MAP_INTERFACE_HPP
