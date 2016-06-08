#include "singlekalmanfilter.h"

SingleKalmanFilter::SingleKalmanFilter()
{
    unsigned int type = CV_32F;
    kf = new cv::KalmanFilter(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf->transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf->measurementMatrix.at<float>(0) = 1.0f;
    kf->measurementMatrix.at<float>(7) = 1.0f;
    kf->measurementMatrix.at<float>(16) = 1.0f;
    kf->measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf->processNoiseCov, cv::Scalar(1e-2));
    kf->processNoiseCov.at<float>(0) = 1e-2;
    kf->processNoiseCov.at<float>(7) = 1e-2;
    kf->processNoiseCov.at<float>(14) = 15.0f;
    kf->processNoiseCov.at<float>(21) = 15.0f;
    kf->processNoiseCov.at<float>(28) = 1e-2;
    kf->processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-1));
    kf->transitionMatrix = (cv::Mat_<float>(6, 6) << 1,0,1,0,0.5,0, 0,1,0,1,0,0.5,  0,0,1,0,1,0, 0,0,0,1,0,1, 0,0,0,0,1,0, 0,0,0,0,0,1);


    // <<<< Kalman Filter
}

void SingleKalmanFilter::setTransitionMatrix(int i, const float &value)
{
    kf->transitionMatrix.at<float>(i) = value;
}

void SingleKalmanFilter::setErrorCov(int i, const float &value)
{
    kf->errorCovPre.at<float>(i) = value;
}

void SingleKalmanFilter::setStatePre(const cv::Rect &bbox)
{
    kf->statePre.at<float>(0) = bbox.x+bbox.width/2;
    kf->statePre.at<float>(1) = bbox.y+bbox.height/2;
    kf->statePre.at<float>(2) = 0;
    kf->statePre.at<float>(3) = 0;
    kf->statePre.at<float>(4) = bbox.width;
    kf->statePre.at<float>(5) = bbox.height;
}

void SingleKalmanFilter::setStatePost(const cv::Rect &bbox)
{
    kf->statePost.at<float>(0) = bbox.x+bbox.width/2;
    kf->statePost.at<float>(1) = bbox.y+bbox.height/2;
    kf->statePost.at<float>(2) = 0;
    kf->statePost.at<float>(3) = 0;
    kf->statePost.at<float>(4) = bbox.width;
    kf->statePost.at<float>(5) = bbox.height;
}

void SingleKalmanFilter::setProcessNoiseCov(int i, const float &value)
{
    kf->processNoiseCov.at<float>(i) = value;
}

cv::Point SingleKalmanFilter::predict()
{
    cv::Mat prediction = kf->predict();
    LastResult=cv::Point(prediction.at<float>(0),prediction.at<float>(1));

    return LastResult;
}

cv::Point SingleKalmanFilter::correct(cv::Rect rect,bool correct)
{
    auto p =calcCenter(rect);
  cv::Mat measurement(measSize,1,CV_32FC1);
  if(correct)
  {
      measurement.at<float>(0) = p.x;  //update using measurements
      measurement.at<float>(1) = p.y;
      measurement.at<float>(2) = rect.width;
      measurement.at<float>(3) = rect.height;
  }

  else
  {
      measurement.at<float>(0) = LastResult.x;  //update using prediction
      measurement.at<float>(1) = LastResult.y;
  }
  cv::Mat estimated = kf->correct(measurement);

  LastResult.x=estimated.at<float>(0);   //update using measurements
  LastResult.y=estimated.at<float>(1);

  return LastResult;
}
