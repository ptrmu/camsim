
#include "kal_run.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

namespace camsim
{
  template<typename Type, int StateDims, int MeasureDims>
  class KalmanFilterX
  {
  public:
    cv::Matx<Type, StateDims, StateDims> transitionMatrix;        // state transition matrix (A)
    cv::Matx<Type, MeasureDims, StateDims> measurementMatrix;     // measurement matrix (H)
    cv::Matx<Type, StateDims, StateDims> processNoiseCov;         // process noise covariance matrix (Q)
    cv::Matx<Type, MeasureDims, MeasureDims> measurementNoiseCov; // measurement noise covariance matrix (R)

    struct Specific
    {
      const KalmanFilterX &kf_;

      cv::Vec<Type, StateDims> statePre;                  // predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
      cv::Vec<Type, StateDims> statePost;                 // corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
      cv::Matx<Type, StateDims, StateDims> errorCovPre;  // priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
      cv::Matx<Type, StateDims, StateDims> errorCovPost; // posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)
      cv::Matx<Type, StateDims, MeasureDims> gain;       // Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)

      // temporary matrices
      cv::Matx<Type, StateDims, StateDims> temp1;
      cv::Matx<Type, MeasureDims, StateDims> temp2;
      cv::Matx<Type, MeasureDims, MeasureDims> temp3;
      cv::Matx<Type, MeasureDims, StateDims> temp4;
      cv::Vec<Type, MeasureDims> temp5;

      Specific(const KalmanFilterX &kf) :
        kf_{kf}
      {}

      const cv::Vec<Type, StateDims> &predict(); //
      const cv::Vec<Type, StateDims> &correct(const cv::Mat &measurement); //
    };
  };

  template<typename Type, int StateDims, int MeasureDims>
  const cv::Vec<Type, StateDims> &KalmanFilterX<Type, StateDims, MeasureDims>::Specific::predict()
  {
    // update the state: x'(k) = A*x(k)
    statePre = kf_.transitionMatrix * statePost;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = kf_.transitionMatrix * errorCovPost;

    // P'(k) = temp1*At + Q
    cv::gemm(temp1, kf_.transitionMatrix, 1, kf_.processNoiseCov, 1, errorCovPre, cv::GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    return statePre;
  }

  template<typename Type, int StateDims, int MeasureDims>
  const cv::Vec<Type, StateDims> &KalmanFilterX<Type, StateDims, MeasureDims>::Specific::correct(
    const cv::Mat &measurement)
  {
    // temp2 = H*P'(k)
    temp2 = kf_.measurementMatrix * errorCovPre;

    // temp3 = temp2*Ht + R
    cv::gemm(temp2, kf_.measurementMatrix, 1, kf_.measurementNoiseCov, 1, temp3, cv::GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, cv::DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - kf_.measurementMatrix * statePre;

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain * temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - gain * temp2;

    return statePost;
  }

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

    cv::Mat img_{Rows, Cols, CV_8UC3};
    cv::Point2f center{Cols * 0.5f, Rows * 0.5f};

    cv::Point calcPoint(double R, double angle); //
    void draw_cross(cv::Point2f center, cv::Scalar color); //

  public:
    void show(float state, float predict, float measure, float update);
  };

  template<int Cols, int Rows>
  cv::Point Visualizer<Cols, Rows>::calcPoint(double R, double angle)
  {
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
    cv::Point statePt0 = calcPoint(center, R0, state);
    cv::Point statePt1 = calcPoint(center, R1, state);
    cv::Point statePt2 = calcPoint(center, R2, state);

    cv::Point predictPt = calcPoint(center, R0, predict);
    cv::Point measurePt = calcPoint(center, R1, measure);
    cv::Point updatePt = calcPoint(center, R2, update);

    // plot points
    img_ = cv::Scalar::all(0);
    draw_cross(statePt0, cv::Scalar(255, 255, 255), 3);
    draw_cross(statePt1, cv::Scalar(255, 255, 255), 3);
    draw_cross(statePt2, cv::Scalar(255, 255, 255), 3);
    draw_cross(predictPt, cv::Scalar(0, 255, 255), 3);
    draw_cross(measurePt, cv::Scalar(0, 0, 255), 3);
    draw_cross(updatePt, cv::Scalar(0, 255, 0), 3);
    cv::line(img_, statePt0, predictPt, cv::Scalar(0, 255, 255), 3, cv::LINE_AA, 0);
    cv::line(img_, statePt1, measurePt, cv::Scalar(0, 0, 255), 3, cv::LINE_AA, 0);
    cv::line(img_, statePt2, updatePt, cv::Scalar(0, 255, 0), 3, cv::LINE_AA, 0);

    imshow("Kalman", img_);
  }

  void do_filter_x()
  {
    Visualizer<500, 500> vis{};
    KalmanFilterX<float, 2, 1> kf{};
    KalmanFilterX<float, 2, 1>::Specific filter{kf};
  }
}
