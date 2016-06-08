#ifndef OBJECT_H
#define OBJECT_H

#include "singlekalmanfilter.h"
#include <qlist.h>
class Object
{
public:
    Object( cv::Rect _bbox);
    void updateAge();
    void updateVisibility();
    void updateInvisibility();
    void predict();
    void correct(const cv::Rect &rect,bool cor);

    int getInvisibleCount() const;
    cv::Rect bbox;
    void setCenter(const cv::Point &value);

    cv::Point getCenter();
    int id;
    int  age, totalVisibleCount, invisibleCount;
    cv::Point center,prediction;
    SingleKalmanFilter kf;
    double distance = 0;
private:

    int stateSize = 6;
    int measSize = 4;
    unsigned int type = CV_32F;



    static int newID ;
    cv::Mat state;
    cv::Mat meas;


};

#endif // OBJECT_H
