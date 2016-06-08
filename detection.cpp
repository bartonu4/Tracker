#include "detection.h"

Detection::Detection()
{

file = new QFile("result.log");
  if(!file->open(QIODevice::Text|QIODevice::WriteOnly))
      return;

      stream = new QTextStream(file);
}
float euclideanDist(const cv::Point& p, const cv::Point& q) {
    cv::Point diff = p - q;
    return sqrt(diff.x*diff.x + diff.y*diff.y);
}
cv::Point calcCenter(cv::Rect box)
{
    return cv::Point(box.x+box.width/2,box.y+box.height/2);
}
cv::Ptr<cv::BackgroundSubtractor>  Detection::pMOG= cv::createBackgroundSubtractorMOG2() ;
vector<Object> Detection::detectObjects(cv::Mat frame)
{
    cv::Mat blur;
    cv::imshow("orig",frame);
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0, 0);
    // <<<<< Noise smoothing

    // >>>>> HSV conversion
    cv::Mat frmHsv;
    cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
    // <<<<< HSV conversion


    // >>>>> Color Thresholding
    // Note: change parameters for different colors
    cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::inRange(frmHsv, cv::Scalar(100 , 100, 80),cv::Scalar(150, 255, 255), rangeRes);
    // <<<<< Color Thresholding

    // >>>>> Improving the result
    cv::Mat fgMaskMOG;
    // <<<<< Improving the result

    pMOG->apply(frmHsv,fgMaskMOG);

    int  morph_size = 2;
    cv::Mat opElement = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    morph_size = 2;
    cv::Mat clElement = getStructuringElement( cv::MORPH_ELLIPSE, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    for (int i = 0;i<3;i++)
    {

        cv::morphologyEx( fgMaskMOG, fgMaskMOG, cv::MORPH_OPEN, opElement, cv::Point(-1,-1), i );

        cv::morphologyEx( fgMaskMOG, fgMaskMOG, cv::MORPH_CLOSE, clElement, cv::Point(-1,-1), i );


    }


    // Thresholding f


    cv::imshow("Threshold", fgMaskMOG);

    //Create parameters for Harris corner

    vector<cv::Point2f> features;

    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(fgMaskMOG.size(), CV_32FC1);
    int blockSize = 7;
    int apertureSize = 5;
    double k = 0.05;
    // cornerHarris(fgMaskMOG, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
    cv::Canny(fgMaskMOG,dst,50,190,3);

    cv::goodFeaturesToTrack(dst, features,1500,0.05,20,cv::noArray(),3,false);
    // Normalizing
    //        normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    //        convertScaleAbs( dst_norm, dst_norm_scaled );

    //        // Drawing a circle around corners
    //        vector<Point> box;
    //        for( int j = 0; j < dst_norm.rows ; j++ )
    //        {
    //            for( int i = 0; i < dst_norm.cols; i++ )
    //            {
    //                if( (int) dst_norm.at<float>(j,i) > 220 )
    //                {
    //                    circle( dst_norm_scaled, Point( i, j ), 8,  Scalar(120), 2, 8, 0 )ZZ;

    //                }

    //            }
    //        }

    for( uint j = 0; j < features.size() ; j++ )
    {
        cv::circle(dst,features.at(j),8,cv::Scalar(120),2);
    }





    cv::imshow("canny", dst);
    // >>>>> Contours detection
    vector<vector<cv::Point> > contours;
    cv::findContours(dst, contours, CV_RETR_EXTERNAL,
                     CV_CHAIN_APPROX_SIMPLE);
    // <<<<< Contours detection


    // >>>>> Filtering
    vector<vector<cv::Point> > balls;

    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Rect bBox;
        vector<cv::Point> contour;
        cv::approxPolyDP(contours[i],contour,0.05,true);
        bBox = cv::boundingRect(contour);
        ballsBox.push_back(bBox);
//        float ratio = (float) bBox.width / (float) bBox.height;
//        if (ratio > 1.0f)
//            ratio = 1.0f / ratio;

//        // Searching for a bBox almost square
//        if (ratio > 0.75 && bBox.area() >= 200)
//        {
//            balls.push_back(contours[i]);
//
//        }
    }
if(objects.empty())
{
    for (size_t i = 0; i < ballsBox.size(); i++)
    {
        objects.push_back(Object(ballsBox[i]));
      //  cv::rectangle(frame,ballsBox[i],cv::Scalar(0,255,0),4);


    }
}
    first++;
    cv::Mat res_frame;
    res_frame = frame.clone();
    if(objects.size()>0)
    {
        assignObjects();
    }
    for(int i =0;i<objects.size();i++)
    {
        std::stringstream str;
        str<<objects[i].id;
        if(objects[i].age>=10)
        {

            ballsBox.push_back(objects[i].bbox);

            //cv::putText(res_frame,str.str(),objects[i].center,2,1,cv::Scalar(255,0,0));
        }

     // std::cout<<objects[i].id;

    }
    for (size_t i = 0; i < ballsBox.size(); i++)
    {

      cv::rectangle(frame,ballsBox[i],cv::Scalar(0,255,0),2);

 *stream <<"bbox center " <<ballsBox[i].x<< " "<<ballsBox[i].y<<"\n";
    }
   *stream <<"\nnew frame\n\n ";
    ballsBox.clear();
    cv::imshow("objects",frame);

    return objects;
}
void showObj(QTextStream &stream,Object &o)
{
    stream<<"Object id="<<o.id<<" age = "<<o.age <<" visibility"<<o.totalVisibleCount<<"\n\n";

}
void Detection::assignObjects()
{

    if(first==0)
    {
        return;
    }
    else
    {
        QVector<int> newObjects,visible,invisble;
        QVector<float> dist;
        cv::Mat distance(objects.size(),ballsBox.size(),CV_32F);
        bool vis =false;
        float min =0;
        int mInd = 0;
        for(int i =0;i<objects.size();i++)
        {
            for(int j =0;j<ballsBox.size();j++)
            {
                float d = euclideanDist(objects[i].getCenter(),calcCenter(ballsBox[j]));
                dist.push_back(d);
                distance.at<float>(i,j)=d;
                if(d<100)
                {
                    vis = true;
                    if(min>d)
                    {
                        min = d;
                        mInd = j;
                    }

                }


            }

            if(vis==false )
            {
               objects.at(i).updateInvisibility();
               showObj(*stream,objects.at(i));
            }
            else
            {
                 objects.at(i).updateVisibility();
                 objects[i].bbox = ballsBox[mInd];
                  min =0;
                  mInd = 0;
            }
        }

        bool isNewObj=true;

        for(int j =0;j<ballsBox.size();j++)
        {
            for(int i =0;i<objects.size();i++)
            {
                if(distance.at<float>(i,j)<80)
                {
                    isNewObj = false;
                }


            }
            if(isNewObj ==true)
            {
               auto obj = Object(ballsBox.at(j));
                showObj(*stream,obj);
                objects.push_back(obj);
            }
            isNewObj = true;
        }


        for(int i =0;i<objects.size();i++)
        {
            if(objects[i].getInvisibleCount()>=maximum_allow_invis)
            {
                objects.erase(objects.begin()+i);
                i--;
            }

           // std::cout<<objects[i].id;

        }
        ballsBox.clear();

    }

}
