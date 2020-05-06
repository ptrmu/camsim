
#include "kal_run.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui.hpp"

#include <iostream>

namespace camsim
{

  using namespace cv;


  class CV_EXPORTS_W KalmanFilterPriv
  {
  public:
    CV_WRAP KalmanFilterPriv();
    /** @overload
    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
    */
    CV_WRAP KalmanFilterPriv(int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F);

    /** @brief Re-initializes Kalman filter. The previous content is destroyed.

    @param dynamParams Dimensionality of the state.
    @param measureParams Dimensionality of the measurement.
    @param controlParams Dimensionality of the control vector.
    @param type Type of the created matrices that should be CV_32F or CV_64F.
     */
    void init(int dynamParams, int measureParams, int controlParams = 0, int type = CV_32F);

    /** @brief Computes a predicted state.

    @param control The optional input control
     */
    CV_WRAP const Mat &predict(const Mat &control = Mat());

    /** @brief Updates the predicted state from the measurement.

    @param measurement The measured system parameters
     */
    CV_WRAP const Mat &correct(const Mat &measurement);

    CV_PROP_RW Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k)
    CV_PROP_RW Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k))
    CV_PROP_RW Mat transitionMatrix;   //!< state transition matrix (A)
    CV_PROP_RW Mat controlMatrix;      //!< control matrix (B) (not used if there is no control)
    CV_PROP_RW Mat measurementMatrix;  //!< measurement matrix (H)
    CV_PROP_RW Mat processNoiseCov;    //!< process noise covariance matrix (Q)
    CV_PROP_RW Mat measurementNoiseCov;//!< measurement noise covariance matrix (R)
    CV_PROP_RW Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)): P'(k)=A*P(k-1)*At + Q)*/
    CV_PROP_RW Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)
    CV_PROP_RW Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k)

    // temporary matrices
    Mat temp1;
    Mat temp2;
    Mat temp3;
    Mat temp4;
    Mat temp5;
  };


  KalmanFilterPriv::KalmanFilterPriv()
  {}

  KalmanFilterPriv::KalmanFilterPriv(int dynamParams, int measureParams, int controlParams, int type)
  {
    init(dynamParams, measureParams, controlParams, type);
  }

  void KalmanFilterPriv::init(int DP, int MP, int CP, int type)
  {
    CV_Assert(DP > 0 && MP > 0);
    CV_Assert(type == CV_32F || type == CV_64F);
    CP = std::max(CP, 0);

    statePre = Mat::zeros(DP, 1, type);
    statePost = Mat::zeros(DP, 1, type);
    transitionMatrix = Mat::eye(DP, DP, type);

    processNoiseCov = Mat::eye(DP, DP, type);
    measurementMatrix = Mat::zeros(MP, DP, type);
    measurementNoiseCov = Mat::eye(MP, MP, type);

    errorCovPre = Mat::zeros(DP, DP, type);
    errorCovPost = Mat::zeros(DP, DP, type);
    gain = Mat::zeros(DP, MP, type);

    if (CP > 0)
      controlMatrix = Mat::zeros(DP, CP, type);
    else
      controlMatrix.release();

    temp1.create(DP, DP, type);
    temp2.create(MP, DP, type);
    temp3.create(MP, MP, type);
    temp4.create(MP, DP, type);
    temp5.create(MP, 1, type);
  }

  const Mat &KalmanFilterPriv::predict(const Mat &control)
  {
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix * statePost;

    if (!control.empty())
      // x'(k) = x'(k) + B*u(k)
      statePre += controlMatrix * control;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix * errorCovPost;

    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    return statePre;
  }

  const Mat &KalmanFilterPriv::correct(const Mat &measurement)
  {
    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // temp3 = temp2*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix * statePre;

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain * temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - gain * temp2;

    return statePost;
  }

#define drawCross(center, color, d)                                        \
                line( img, Point( center.x - d, center.y - d ),                          \
                             Point( center.x + d, center.y + d ), color, 1, LINE_AA, 0); \
                line( img, Point( center.x + d, center.y - d ),                          \
                             Point( center.x - d, center.y + d ), color, 1, LINE_AA, 0 )


  static inline Point calcPoint(Point2f center, double R, double angle)
  {
    return center + Point2f((float) cos(angle), (float) -sin(angle)) * (float) R;
  }

  static void help()
  {
    printf("\nExample of c calls to OpenCV's Kalman filter.\n"
           "   Tracking of rotating point.\n"
           "   Rotation speed is constant.\n"
           "   Both state and measurements vectors are 1D (a point angle),\n"
           "   Measurement is the real point angle + gaussian noise.\n"
           "   The real and the estimated points are connected with yellow line segment,\n"
           "   the real and the measured points are connected with red line segment.\n"
           "   (if Kalman filter works correctly,\n"
           "    the yellow segment should be shorter than the red one).\n"
           "\n"
           "   Pressing any key (except ESC) will reset the tracking with a different speed.\n"
           "   Pressing ESC will stop the program.\n"
    );
  }

  int do_filter()
  {
    help();
    Mat img(500, 500, CV_8UC3);
    KalmanFilterPriv KF(2, 1, 0);
    Mat state(2, 1, CV_32F); /* (phi, delta_phi) */
    Mat processNoise(2, 1, CV_32F);
    Mat measurement = Mat::zeros(1, 1, CV_32F);
    char code = (char) -1;

    for (;;) {
      randn(state, Scalar::all(0), Scalar::all(0.1));
      KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

      setIdentity(KF.measurementMatrix);
      setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
      setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
      setIdentity(KF.errorCovPost, Scalar::all(1));

      randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

      for (;;) {
        Point2f center(img.cols * 0.5f, img.rows * 0.5f);
        float R = img.cols / 3.f;
        double stateAngle = state.at<float>(0);
        Point statePt = calcPoint(center, R, stateAngle);

        Mat prediction = KF.predict();
        double predictAngle = prediction.at<float>(0);
        Point predictPt = calcPoint(center, R, predictAngle);

        randn(measurement, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));

        // generate measurement
        measurement += KF.measurementMatrix * state;

        double measAngle = measurement.at<float>(0);
        Point measPt = calcPoint(center, R, measAngle);

        // plot points
        img = Scalar::all(0);
        drawCross(statePt, Scalar(255, 255, 255), 3);
        drawCross(measPt, Scalar(0, 0, 255), 3);
        drawCross(predictPt, Scalar(0, 255, 0), 3);
        line(img, statePt, measPt, Scalar(0, 0, 255), 3, LINE_AA, 0);
        line(img, statePt, predictPt, Scalar(0, 255, 255), 3, LINE_AA, 0);

        if (theRNG().uniform(0, 4) != 0)
          KF.correct(measurement);

        randn(processNoise, Scalar(0), Scalar::all(sqrt(KF.processNoiseCov.at<float>(0, 0))));
        state = KF.transitionMatrix * state + processNoise;

        imshow("Kalman", img);
        code = (char) waitKey(100);

        if (code > 0)
          break;
      }
      if (code == 27 || code == 'q' || code == 'Q')
        break;
    }

    return 0;
  }

  void next_model_state(Mat & model)
  {
    float delta_phi = 0.1;
    static int loop_count{0};
    switch (loop_count++ / 25 % 5)
    {
      default:
      case 0:
        model.at<float>(1) = 0;
        break;
      case 1:
        model.at<float>(1) = delta_phi;
        break;
      case 2:
        model.at<float>(1) = - 2 * delta_phi;
        break;
      case 3:
        model.at<float>(1) = delta_phi;
        break;
    }
    model.at<float>(0) += model.at<float>(1);
  }

  int do_filter_model()
  {
    help();
    Mat img(500, 500, CV_8UC3);
    KalmanFilterPriv KF(2, 1, 0);

    Mat model(2, 1, CV_32F); /* (phi, delta_phi) */

    Mat processNoise(2, 1, CV_32F);
    Mat measurementNoise(1, 1, CV_32F);

    char code = (char) -1;

    Point2f center(img.cols * 0.5f, img.rows * 0.5f);
    float R0 = img.cols / 3.f;
    float R1 = R0 * 1.1;
    float R2 = R0 * 1.2;

    for (;;) {
      randn(model, Scalar::all(0), Scalar::all(0.1));

      KF.transitionMatrix = (Mat_<float>(2, 2) << 1, 1, 0, 1);

      setIdentity(KF.measurementMatrix);
      setIdentity(KF.processNoiseCov, Scalar::all(1e-1));
      setIdentity(KF.measurementNoiseCov, Scalar::all(2e-1));
      setIdentity(KF.errorCovPost, Scalar::all(1));

      randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

      for (;;) {
        // Generate the new model state
        next_model_state(model);

        // Get the predicted state
        Mat prediction = KF.predict();

        // Get the measurement as the model with noise.
        randn(measurementNoise, Scalar::all(0), Scalar::all(KF.measurementNoiseCov.at<float>(0)));
        Mat measurement = KF.measurementMatrix * model + measurementNoise;

        // Correct the prediction with the measurement
        KF.correct(measurement);

        // Display stuff
        Point statePt0 = calcPoint(center, R0, model.at<float>(0));
        Point statePt1 = calcPoint(center, R1, model.at<float>(0));
        Point statePt2 = calcPoint(center, R2, model.at<float>(0));

        Point predictPt = calcPoint(center, R0, prediction.at<float>(0));
        Point measPt = calcPoint(center, R1, measurement.at<float>(0));
        Point correctPt = calcPoint(center, R2, KF.statePost.at<float>(0));

        // plot points
        img = Scalar::all(0);
        drawCross(statePt0, Scalar(255, 255, 255), 3);
        drawCross(statePt1, Scalar(255, 255, 255), 3);
        drawCross(statePt2, Scalar(255, 255, 255), 3);
        drawCross(predictPt, Scalar(0, 255, 255), 3);
        drawCross(measPt, Scalar(0, 0, 255), 3);
        drawCross(correctPt, Scalar(0, 255, 0), 3);
        line(img, statePt0, predictPt, Scalar(0, 255, 255), 3, LINE_AA, 0);
        line(img, statePt1, measPt, Scalar(0, 0, 255), 3, LINE_AA, 0);
        line(img, statePt2, correctPt, Scalar(0, 255, 0), 3, LINE_AA, 0);

        imshow("Kalman", img);
        code = (char) waitKey(100);

        if (code > 0)
          break;
      }
      if (code == 27 || code == 'q' || code == 'Q')
        break;
    }

    return 0;
  }


  void kal_opencv_filter()
  {
    do_filter_model();
  }
}
