#include "object.h"

Object::Object( cv::Rect _bbox):bbox(_bbox)
{
    state.create(stateSize,1,type);
    meas.create(measSize,1,type);
    id=newID;
    newID++;
    age = 0;
    totalVisibleCount = 0;
    invisibleCount = 0;
    center.x = bbox.x +bbox.width/2;
    center.y = bbox.y +bbox.height/2;
    kf.setStatePre(bbox);
    kf.setStatePost(bbox);
    kf.predict();
}
int Object::newID = 0;
void Object::updateAge()
{
    age++;
}

void Object::updateVisibility()
{
    updateAge();
    totalVisibleCount++;
}

void Object::updateInvisibility()
{
    updateAge();
    invisibleCount++;
}

void Object::predict()
{


    // <<<< Initialization

    auto p = kf.predict();
    bbox.x = p.x;
    bbox.y = p.y;
}

void Object::correct(const cv::Rect &rect, bool cor)
{/*
    meas.at<float>(0) = bbox.x + bbox.width / 2;
    meas.at<float>(1) = bbox.y + bbox.height / 2;
    meas.at<float>(2) = (float)bbox.width;
    meas.at<float>(3) = (float)bbox.height;


    // >>>> Initialization
    kf.setErrorCov(0,1);
    kf.setErrorCov(7,1);
    kf.setErrorCov(14,1);
    kf.setErrorCov(28,1);
    kf.setErrorCov(35,1);


    state.at<float>(0) = meas.at<float>(0);
    state.at<float>(1) = meas.at<float>(1);
    state.at<float>(2) = 0;
    state.at<float>(3) = 0;
    state.at<float>(4) = meas.at<float>(2);
    state.at<float>(5) = meas.at<float>(3);*/
    prediction =  kf.correct(rect,cor);
}

int Object::getInvisibleCount() const
{
    return invisibleCount;
}

void Object::setCenter(const cv::Point &value)
{
    center = value;
}

cv::Point Object::getCenter()
{
    center = calcCenter(bbox);
    return center;
}
