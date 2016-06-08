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
    cv::Point predict();
    cv::Point correct(cv::Rect rect, bool correct);
    cv::Point LastResult;

};
inline cv::Point calcCenter(cv::Rect box)
{
    return cv::Point(box.x+box.width/2,box.y+box.height/2);
}
#endif // SINGLEKALMANFILTER_H
