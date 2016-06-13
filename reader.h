#ifndef READER_H
#define READER_H

#include <QObject>
#include <QtWidgets/qfiledialog.h>
#include <opencv2/core.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/videoio.hpp"
#include <iostream>
class Reader : public QObject
{
    Q_OBJECT
    QApplication *a;
    cv::VideoCapture cap;
     int frameCounter;
public:
    explicit Reader(QObject *parent = 0);
    cv::Point openFromFile();
    cv::Point openCam();
    cv::Mat  getFrame();
    QString filename;
signals:

public slots:
};

#endif // READER_H
