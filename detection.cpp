#include "detection.h"

void Detection::calcMaxDistance(int width, int height)
{
    dist_thres = sqrtf(width*width+height*height)*0.1;
    minArea = width*height*0.001;
    //minArea =400;
}

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

int history = 200;
int threshold = history*0.7;
cv::Ptr<cv::BackgroundSubtractor>  Detection::pMOG= cv::createBackgroundSubtractorMOG2(history,threshold,false) ;
cv::Mat Detection::detectObjects(cv::Mat frame)
{
    cv::Mat blur,filter;
    cv::Point size(frame.cols,frame.rows);
    cv::Mat allImages(size.y*2+10,size.x*2+10,frame.type());
    cv::Rect roi(0,0,size.x,size.y);
    //cv::Mat roi(all,cv::Range(0,0+frame.cols),cv::Range(0,0+frame.rows));

    frame.copyTo(allImages(roi));




    //cv::imshow("orig",allImages);
    cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0, 0);
    //cv::bilateralFilter ( blur, filter, 15, 80, 80 );
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
    //cv::fastNlMeansDenoising(fgMaskMOG,fgMaskMOG);
    auto element = cv::MORPH_ELLIPSE;
    int  morph_size = 3;
    cv::Mat opElement = getStructuringElement( element, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    morph_size = 7;
    cv::Mat clElement = getStructuringElement( element, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) );
    for (int i = 0;i<5;i++)
    {

        cv::morphologyEx( fgMaskMOG, fgMaskMOG, cv::MORPH_OPEN, opElement, cv::Point(-1,-1), 1 );

        cv::morphologyEx( fgMaskMOG, fgMaskMOG, cv::MORPH_CLOSE, clElement, cv::Point(-1,-1), 1 );


    }


    // Thresholding f


    //cv::imshow("Threshold", fgMaskMOG);
    roi = cv::Rect(size.x+10,0,size.x,size.y);
    cv::Mat mask;

    cv::cvtColor(fgMaskMOG,mask,CV_GRAY2BGR);
    mask.convertTo(allImages(roi),allImages.type());

    //Create parameters for Harris corner

    vector<cv::Point2f> features;

    cv::Mat dst, dst_norm;
    dst = cv::Mat::zeros(fgMaskMOG.size(), CV_32FC1);


    // cornerHarris(fgMaskMOG, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
    cv::Canny(fgMaskMOG,dst,50,250,3);


    for( uint j = 0; j < features.size() ; j++ )
    {
        //cv::circle(dst,features.at(j),8,cv::Scalar(120),2);
    }


    roi = cv::Rect(0,size.y+10,size.x,size.y);
    //cv::Mat m
    dst.copyTo(dst_norm);
    cv::cvtColor(dst_norm,mask,CV_GRAY2BGR);
    mask.convertTo(allImages(roi),allImages.type());




   // cv::imshow("canny", dst);
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
        if(bBox.area()>minArea)
        {
             ballsBox.push_back(bBox);
        }

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
        if(objects[i].totalVisibleCount>=3)
        {



            cv::rectangle(frame,objects[i].bbox,cv::Scalar(0,255,0),2);

            cv::putText(frame,str.str(),cv::Point(objects[i].bbox.x,objects[i].bbox.y),2,1,cv::Scalar(255,0,0));
        }

     // std::cout<<objects[i].id;

    }

    ballsBox.clear();

    roi = cv::Rect(size.x+10,size.y+10,size.x,size.y);
    frame.copyTo(allImages(roi));
    cv::line(allImages,cv::Point(0,size.y),cv::Point(allImages.cols,size.y),cv::Scalar(255,255,255),10);
    cv::line(allImages,cv::Point(size.x,0),cv::Point(size.x,allImages.rows),cv::Scalar(255,255,255),10);

    cv::imshow("objects",allImages);

    return allImages;
}
void showObj(QTextStream &stream,Object &o)
{
    stream<<"Object id="<<o.id<<" age = "<<o.age <<" visibility"<<o.totalVisibleCount<<"\n\n";

}



void Detection::assignObjects()
{


        QVector<int> newObjects,visible,invisble;
        QVector<float> dist;
        vector< vector<double> > Cost(objects.size(),vector<double>(ballsBox.size()));
        vector<int> assignment;
        bool vis =false;
        float min =0;
        int mInd = 0;

        for(int i =0;i<objects.size();i++)
              {
                  for(int j =0;j<ballsBox.size();j++)
                  {
                      double d = euclideanDist(objects[i].getCenter(),calcCenter(ballsBox[j]));
                      dist.push_back(d);
                      Cost[i][j]= d;
                  }
        }
        AssignmentProblemSolver APS;
        APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);

        vector<int> not_assigned_tracks;

        for(int i=0;i<assignment.size();i++)
        {
            if(assignment[i]!=-1)
            {
                if(Cost[i][assignment[i]]>dist_thres)
                {
                    assignment[i]=-1;
                    // Mark unassigned tracks, and increment skipped frames counter,
                    // when skipped frames counter will be larger than threshold, track will be deleted.
                    not_assigned_tracks.push_back(i);
                }
                objects[i].distance+= Cost[i][assignment[i]];

                objects[i].totalVisibleCount++;
            }
            else
            {
                // If track have no assigned detect, then increment skipped frames counter.
                objects[i].invisibleCount++;
            }

        }

        // -----------------------------------
        // If track didn't get detects long time, remove it.
        // -----------------------------------
        for(int i=0;i<objects.size();i++)
        {
            if(objects[i].invisibleCount>maximum_allow_invis||objects[i].distance<1)
            {

                objects.erase(objects.begin()+i);
                assignment.erase(assignment.begin()+i);
                i--;
            }
        }
        // -----------------------------------
        // Search for unassigned detects
        // -----------------------------------
        vector<int> not_assigned_detections;
        vector<int>::iterator it;
        for(int i=0;i<ballsBox.size();i++)
        {
            it=find(assignment.begin(), assignment.end(), i);
            if(it==assignment.end())
            {
                not_assigned_detections.push_back(i);
            }
        }

        // -----------------------------------
        // and start new tracks for them.
        // -----------------------------------
        if(not_assigned_detections.size()!=0)
        {
            for(int i=0;i<not_assigned_detections.size();i++)
            {
                auto obj = Object(ballsBox[not_assigned_detections[i]]);
                objects.push_back(obj);
            }
        }
        for(int i=0;i<assignment.size();i++)
        {
            // If track updated less than one time, than filter state is not correct.

            objects[i].predict();

            if(assignment[i]!=-1) // If we have assigned detect, then update using its coordinates,
            {
                objects[i].invisibleCount=0;
                objects[i].correct(ballsBox[assignment[i]],1);
            }
            else				  // if not continue using predictions
            {
                objects[i].kf.correct(cv::Rect(cv::Point(0,0),cv::Point(0,0)),0);
            }
            objects[i].kf.LastResult = calcCenter(objects[i].bbox);
         cv::Rect &box = objects[i].bbox;
         box.x = objects[i].prediction.x-box.width/2;
         box.y = objects[i].prediction.y-box.height/2;

        }




}
