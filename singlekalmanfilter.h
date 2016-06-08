#ifndef SINGLEKALMANFILTER_H
#define SINGLEKALMANFILTER_H
#include <opencv2/core.hpp>
// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>
#include "opencv2/videoio.hpp"
#include<opencv2/video/tracking.hpp>

class SingleKalmanFilter
{
private:
    cv::KalmanFilter *kf;
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
public:
    SingleKalmanFilter();
    void setTransitionMatrix(int i,const float &value);
    void setProcessNoiseCov(int i, const float &value);
    void setErrorCov(int i, const float &value);
    void setStatePre(const cv::Rect &bbox);
    void setStatePost(const cv::Rect &bbox);
    const cv::Mat &predict();
    const cv::Mat & correct(const cv::Mat &measurments);

};

#endif // SINGLEKALMANFILTER_H
