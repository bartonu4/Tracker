#include "reader.h"

Reader::Reader(QObject *parent) : QObject(parent)
{
frameCounter = 0;
}

cv::VideoCapture Reader::openFromFile()
{
    QString path = QFileDialog::getOpenFileName(new QWidget(), tr("Find Files"), QString("F:/test3.avi"));


    cap.open(path.toStdString());
    if(!cap.isOpened())
    {
        std::cout <<"eror opening video";
    }
    return cap;
}

cv::VideoCapture Reader::openCam()
{
    return cap;
}

cv::Mat Reader::getFrame()
{

    if(!cap.isOpened())
    {
       openFromFile();
       cv::Mat frame;
       cap>>frame;
       frameCounter++;
       return frame;
    }
    else
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

}
