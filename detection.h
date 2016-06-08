#ifndef DETECTION_H
#define DETECTION_H

#include "object.h"
#include <iostream>
#include "vector"
#include <QVector>
#include <qdebug.h>
#include <QFile>
#include <HungarianAlg.h>
#include <opencv2/photo.hpp>
using std::vector;
class Detection
{
    static  cv::Ptr<cv::BackgroundSubtractor>  pMOG ;
    vector<Object> objects;
    int first=0;
    int maximum_allow_invis = 1;
    int dist_thres = 120;
   vector<cv::Rect> ballsBox;
   QFile *file;
   QTextStream *stream;

public:
    Detection();
     vector<Object> detectObjects(cv::Mat frame);
     void assignObjects();

};

#endif // DETECTION_H
