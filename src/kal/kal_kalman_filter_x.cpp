
#include "kal_run.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

namespace camsim
{

// ==============================================================================
// KalmanFilterX class
// ==============================================================================

  // A class the implements a kalman filter. This filter is implemented using
  // opencv's Matx matricies. The implementation followes the opencv KalmanFilter
  // implementation.

  template<typename Type, int StateDims, int MeasurementDims>
  class KalmanFilterX
  {
  public:
    using StateType = cv::Vec<Type, StateDims>;
    using MeasurementType = cv::Vec<Type, MeasurementDims>;
    using StateCovType = cv::Matx<Type, StateDims, StateDims>;
    using MeasurementCovType = cv::Matx<Type, MeasurementDims, MeasurementDims>;

  private:
    // temporary matrices
    cv::Matx<Type, StateDims, StateDims> temp1;
    cv::Matx<Type, MeasurementDims, StateDims> temp2;
    cv::Matx<Type, MeasurementDims, MeasurementDims> temp3;
    cv::Matx<Type, MeasurementDims, StateDims> temp4;
    cv::Vec<Type, MeasurementDims> temp5;

    StateType x_k_prime;                              // predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    StateCovType error_cov_k_prime;                   // priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    cv::Matx<Type, StateDims, MeasurementDims> gain;  // Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)

  public:
    cv::Matx<Type, StateDims, StateDims> transition_matrix;         // state transition matrix (A)
    cv::Matx<Type, MeasurementDims, StateDims> measurement_matrix;  // measurement matrix (H)
    StateCovType process_noise_cov;                                 // process noise covariance matrix (Q)
    MeasurementCovType measurement_noise_cov;                       // measurement noise covariance matrix (R)

    struct FilterState
    {
      StateType x_k;                                  // corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
      StateCovType error_cov_k;                       // posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
    };

    const StateType &predict(FilterState &fs); //
    const StateType &update(FilterState &fs,
                            const MeasurementType &measurement); //
  };

  template<typename Type, int StateDims, int MeasureDims>
  const typename KalmanFilterX<Type, StateDims, MeasureDims>::StateType &
  KalmanFilterX<Type, StateDims, MeasureDims>::predict(FilterState &fs)
  {
    // update the state: x'(k) = A*x(k)
    x_k_prime = transition_matrix * fs.x_k;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transition_matrix * fs.error_cov_k;

    // P'(k) = temp1*At + Q
    cv::gemm(temp1, transition_matrix, 1, process_noise_cov, 1, error_cov_k_prime, cv::GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    fs.x_k = x_k_prime;
    fs.error_cov_k = error_cov_k_prime;

    return x_k_prime;
  }

  template<typename Type, int StateDims, int MeasureDims>
  const typename KalmanFilterX<Type, StateDims, MeasureDims>::StateType &
  KalmanFilterX<Type, StateDims, MeasureDims>::update(FilterState &d,
                                                      const MeasurementType &measurement)
  {
    // temp2 = H*P'(k)
    temp2 = measurement_matrix * error_cov_k_prime;

    // temp3 = temp2*Ht + R
    cv::gemm(temp2, measurement_matrix, 1, measurement_noise_cov, 1, temp3, cv::GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, cv::DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurement_matrix * x_k_prime;

    // x(k) = x'(k) + K(k)*temp5
    d.x_k = x_k_prime + gain * temp5;

    // P(k) = P'(k) - K(k)*temp2
    d.error_cov_k = error_cov_k_prime - gain * temp2;

    return d.x_k;
  }

// ==============================================================================
// Visualizer class
// ==============================================================================

  template<int Cols, int Rows>
  class Visualizer
  {
    constexpr static float R0 = Cols / 3.f;
    constexpr static float R1 = R0 * 1.1;
    constexpr static float R2 = R0 * 1.2;
    constexpr static int cross_size_ = 3;

    cv::Scalar white{255, 255, 255};
    cv::Scalar red{0, 0, 255};
    cv::Scalar yellow{0, 255, 255};
    cv::Scalar green{0, 255, 0};

    cv::Mat img_;
    cv::Point2f center{Cols * 0.5f, Rows * 0.5f};

    cv::Point calc_point(double R, double angle); //
    void draw_cross(cv::Point2f center, cv::Scalar color); //

  public:
    Visualizer() :
      img_(Rows, Cols, CV_8UC3)
    {

    }

    void show(float state, float predict, float measure, float update);
  };

  template<int Cols, int Rows>
  cv::Point Visualizer<Cols, Rows>::calc_point(double R, double angle)
  {
    angle *= M_PI / 180.;
    return center + cv::Point2f((float) cos(angle), (float) -sin(angle)) * (float) R;
  }

  template<int Cols, int Rows>
  void Visualizer<Cols, Rows>::draw_cross(cv::Point2f center, cv::Scalar color)
  {
    cv::line(img_, cv::Point2f(center.x - cross_size_, center.y - cross_size_),
             cv::Point2f(center.x + cross_size_, center.y + cross_size_), color, 1, cv::LINE_AA, 0);
    cv::line(img_, cv::Point2f(center.x + cross_size_, center.y - cross_size_),
             cv::Point2f(center.x - cross_size_, center.y + cross_size_), color, 1, cv::LINE_AA, 0);
  }

  template<int Cols, int Rows>
  void Visualizer<Cols, Rows>::show(float state, float predict, float measure, float update)
  {
    cv::Point state_0 = calc_point(R0, state);
    cv::Point state_1 = calc_point(R1, state);
    cv::Point state_2 = calc_point(R2, state);

    cv::Point predictPt = calc_point(R0, predict);
    cv::Point measurePt = calc_point(R1, measure);
    cv::Point updatePt = calc_point(R2, update);

    // plot points
    img_ = cv::Scalar::all(0);
    draw_cross(state_0, white);
    draw_cross(state_1, white);
    draw_cross(state_2, white);
    draw_cross(predictPt, yellow);
    draw_cross(measurePt, red);
    draw_cross(updatePt, green);
    cv::line(img_, state_0, predictPt, yellow, 3, cv::LINE_AA, 0);
    cv::line(img_, state_1, measurePt, red, 3, cv::LINE_AA, 0);
    cv::line(img_, state_2, updatePt, green, 3, cv::LINE_AA, 0);

    imshow("Kalman", img_);
  }

// ==============================================================================
// CVFType class
// ==============================================================================

// Constant Velocity Kalman Filter. This class has a 2 element state vector. The
// state variables are position and velocity.

  class CVFType : public KalmanFilterX<float, 2, 1>
  {
  public:
    CVFType() = default; //
    void init(float dt,
              float process_std_dev,
              float measurement_std_dev); //
    void init_filter_state(float measurement_0,
                           FilterState &fs); //
  };

  void CVFType::init(float dt,
                     float process_std_dev,
                     float measurement_std_dev)
  {
    transition_matrix = {1, dt, 0, 1};
    measurement_matrix = {1, 0};
    process_noise_cov = StateCovType{dt * dt, dt, dt, 1.} * process_std_dev * process_std_dev;
    measurement_noise_cov = MeasurementCovType{measurement_std_dev * measurement_std_dev};
  }

  void CVFType::init_filter_state(float measurement_0,
                                  FilterState &fs)
  {
    fs.x_k = StateType{measurement_0, 0.}; // (phi, delta_phi)
    fs.error_cov_k = {10., 10., 10., 10.}; // Large uncertainty to start with.
  }

// ==============================================================================
// get_model routine
// ==============================================================================

  static CVFType::StateType get_model(float t)
  {
    const int divisions = 4;
    const float t_section = 4.;
    const float t_period = t_section * divisions;
    const float deg_per_sec_base = 40.;
    const float deg_per_sec_0 = 0.;
    const float deg_per_sec_1 = deg_per_sec_base;
    const float deg_per_sec_2 = -2 * deg_per_sec_base;
    const float deg_per_sec_3 = deg_per_sec_base;

    float t_sub = t - std::floor(t / t_period) * t_period;
    float div = std::floor(t_sub / t_section);
    float t_div = t_sub - div * t_section;
    float deg;
    float deg_per_sec;
    switch (static_cast<int>(div)) {
      default:
      case 0:
        deg_per_sec = deg_per_sec_0;
        deg = 0;
        break;
      case 1:
        deg_per_sec = deg_per_sec_1;
        deg = deg_per_sec_0;
        break;
      case 2:
        deg_per_sec = deg_per_sec_2;
        deg = deg_per_sec_0 + deg_per_sec_1;
        break;
      case 3:
        deg_per_sec = deg_per_sec_3;
        deg = deg_per_sec_0 + deg_per_sec_1 + deg_per_sec_2;
        break;
    }

    deg *= t_section;
    deg += t_div * deg_per_sec;

    return CVFType::StateType{deg, deg_per_sec};
  }

// ==============================================================================
// kal_do_filter_x routine
// ==============================================================================

  void kal_do_filter_x()
  {
    Visualizer<500, 500> vis{};

    CVFType kf{};
    CVFType::FilterState fs{};

    CVFType::MeasurementType measurement;

    float dt = 1. / 15.;
    float process_std_dev = 6; // velocity standard deviation
    float measurement_std_dev = 2;

    kf.init(dt, process_std_dev, measurement_std_dev);

    for (;;) {
      char code = (char) -1;
      float t = 0.0;

      kf.init_filter_state(get_model(t)(0), fs);

      for (;;) {
        t += dt;

        // Generate the new model state as ground truth.
        auto model = get_model(t);

        // Get the predicted state.
        auto prediction = kf.predict(fs);

        // Get the measurement as the model with noise.
        randn(measurement, cv::Scalar::all(0), cv::Scalar::all(measurement_std_dev * measurement_std_dev));
        measurement += kf.measurement_matrix * model ;

        // Update the prediction with the measurement.
        auto update = kf.update(fs, measurement);

        // Display stuff
        vis.show(model(0), prediction(0), measurement(0), update(0));

        code = (char) cv::waitKey(1000 * dt);
        if (code > 0)
          break;
      }
      if (code == 27 || code == 'q' || code == 'Q')
        break;
    }
  }
}
