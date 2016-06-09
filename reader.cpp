#include "reader.h"

Reader::Reader(QObject *parent) : QObject(parent)
{
frameCounter = 0;
}

cv::Point Reader::openFromFile()
{
    QString path = QFileDialog::getOpenFileName(new QWidget(), tr("Find Files"), QString("f:/test3.avi"));


    cap.open(path.toStdString());
    if(!cap.isOpened())
    {
        std::cout <<"eror opening video";
    }

    return cv::Point(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
}

cv::Point Reader::openCam()
{
    cap.open(0);
   return cv::Point(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
}

cv::Mat Reader::getFrame()
{


        if(frameCounter == cap.get(cv::CAP_PROP_FRAME_COUNT))
        {
            cap.set(cv::CAP_PROP_POS_FRAMES,0);
            frameCounter = 0;
        }

        cv::Mat frame;
        cap>>frame;
        frameCounter++;
        return frame;




}
