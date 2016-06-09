//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/video/background_segm.hpp"
//#include "opencv2/video/tracking.hpp"
//#include <stdio.h>
//#include <string>
//using namespace std;
//using namespace cv;
//Point2f point;

//static void help()
//{
//    printf("\n"
//           "This program demonstrated a simple method of connected components clean up of background subtraction\n"
//           "When the program starts, it begins learning the background.\n"
//           "You can toggle background learning on and off by hitting the space bar.\n"
//           "Call\n"
//           "./segment_objects [video file, else it reads camera 0]\n\n");
//}
//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);

//    }
//}
//int main(int argc, char* argv[]) {    // Variable declaration and initialization        // Iterate until the user hits the Esc key

//    while(true)    {        // Capture the current frame
//        VideoCapture cap;
//        cap.open(0);
//        Mat frame, prevgrayImage, curgrayImage, image;
//        Size windowSize(31,31);
//        Point2d currentPoint = point;
//         bool pointTrackingFlag = true;
//        cap >> frame;
//        TermCriteria terminationCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//        // Check if the frame is empty
//        if(frame.empty())
//            break;
//        // Resize the frame
//        float scalingFactor = 2;
//        resize(frame, frame, Size(), scalingFactor, scalingFactor);
//        // Copy the input frame
//        frame.copyTo(image);
//        // Convert the image to grayscale
//        cvtColor(image, curgrayImage, cv::COLOR_BGR2GRAY);
//        // Check if there are points to track
//        vector<Point2d> trackingPoints[2];

//        if(!trackingPoints[0].empty())
//        {
//            // Status vector to indicate whether the flow for the corresponding features has been found
//            vector<uchar> statusVector;
//            // Error vector to indicate the error for the corresponding feature
//            vector<float> errorVector;
//            // Check if previous image is empty
//            if(prevgrayImage.empty())
//            {                curgrayImage.copyTo(prevgrayImage);
//            }
//            // Calculate the optical flow using Lucas-Kanade algorithm
//            calcOpticalFlowPyrLK(prevgrayImage, curgrayImage, trackingPoints[0], trackingPoints[1], statusVector, errorVector, windowSize, 3, terminationCriteria, 0, 0.001);
//            int count = 0;
//            // Minimum distance between any two tracking points
//            int minDist = 7;

//            for(int i=0; i < trackingPoints[1].size(); i++)
//            {

//                if(pointTrackingFlag)
//                {
//                    /* If the new point is within 'minDist' distance from an existing point, it will not be tracked */
//                    if(norm(currentPoint - trackingPoints[1][i]) <= minDist)
//                    {
//                        pointTrackingFlag = false;                        continue;
//                    }
//                }
//                // Check if the status vector is good
//                if(!statusVector[i])                    continue;
//                trackingPoints[1][count++] = trackingPoints[1][i];
//                // Draw a filled circle for each of the tracking points

//                int radius = 8;
//                int thickness = 2;
//                int lineType = 8;
//                Mat image;
//                circle(image, trackingPoints[1][i], radius,
//                        Scalar(0,255,0), thickness, lineType);
//            }
//            trackingPoints[1].resize(count);
//        }
//        int maxnumPoints = 100;
//          // Refining the location of the feature points
//        if(pointTrackingFlag && trackingPoints[1].size() < maxnumPoints)
//        {
//            vector<Point2f> tempPoints;
//            tempPoints.push_back(currentPoint);
//            // Function to refine the location of the corners to subpixel accuracy.
//                  // Here, 'pixel' refers to the image patch of size 'windowSize' and not the actual image pixel
//            cornerSubPix(curgrayImage, tempPoints, windowSize, cvSize(-1,-1), terminationCriteria);
//            trackingPoints[1].push_back(tempPoints[0]);
//            pointTrackingFlag = false;
//        }                // Display the image with the tracking points
//        String windowname = "lkm";
//        imshow(windowname, image);
//        // Check if the user pressed the Esc key
//        char ch = waitKey(10);        if(ch == 27)
//            break;
//        // Swap the 'points' vectors to update 'previous' to            'current'
//        std::swap(trackingPoints[1], trackingPoints[0]);
//        // Swap the images to update previous image to current image
//        cv::swap(prevgrayImage, curgrayImage);
//    }
//    return 0;
//}

//#include "opencv2/video/tracking.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/videoio/videoio.hpp"
//#include "opencv2/highgui/highgui.hpp"

//#include <iostream>
//#include <ctype.h>

//using namespace cv;
//using namespace std;

//static void help()
//{
//    // print a welcome message, and the OpenCV version
//    cout << "\nThis is a demo of Lukas-Kanade optical flow lkdemo(),\n"
//            "Using OpenCV version " << CV_VERSION << endl;
//    cout << "\nIt uses camera by default, but you can provide a path to video as an argument.\n";
//    cout << "\nHot keys: \n"
//            "\tESC - quit the program\n"
//            "\tr - auto-initialize tracking\n"
//            "\tc - delete all the points\n"
//            "\tn - switch the \"night\" mode on/off\n"
//            "To add/remove a feature point click it\n" << endl;
//}

//Point2f point;
//bool addRemovePt = false;

//static void onMouse( int event, int x, int y, int /*flags*/, void* /*param*/ )
//{
//    if( event == EVENT_LBUTTONDOWN )
//    {
//        point = Point2f((float)x, (float)y);
//        addRemovePt = true;
//    }
//}

//int main( int argc, char** argv )
//{
//    VideoCapture cap;
//    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
//    Size subPixWinSize(10,10), winSize(31,31);

//    const int MAX_COUNT = 5000;
//    bool needToInit = false;
//    bool nightMode = false;

//    cv::CommandLineParser parser(argc, argv, "{@input||}{help h||}");
//    string input = parser.get<string>("@input");
//    if (parser.has("help"))
//    {
//        help();
//        return 0;
//    }
//    if( input.empty() )
//        cap.open(0);
//    else if( input.size() == 1 && isdigit(input[0]) )
//        cap.open(input[0] - '0');
//    else
//        cap.open(input);

//    if( !cap.isOpened() )
//    {
//        cout << "Could not initialize capturing...\n";
//        return 0;
//    }

//    namedWindow( "LK Demo", 1 );
//    setMouseCallback( "LK Demo", onMouse, 0 );

//    Mat gray, prevGray, image, frame;
//    vector<Point2f> points[2];

//    for(;;)
//    {
//        cap >> frame;
//        if( frame.empty() )
//            break;

//        frame.copyTo(image);
//        cvtColor(image, gray, COLOR_BGR2GRAY);

//        if( nightMode )
//            image = Scalar::all(0);

//        if( needToInit )
//        {
//            // automatic initialization
//            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
//            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
//            addRemovePt = false;
//        }
//        else if( !points[0].empty() )
//        {
//            vector<uchar> status;
//            vector<float> err;
//            if(prevGray.empty())
//                gray.copyTo(prevGray);
//            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
//                                 3, termcrit, 0, 0.001);
//            size_t i, k;
//            for( i = k = 0; i < points[1].size(); i++ )
//            {
//                if( addRemovePt )
//                {
//                    if( norm(point - points[1][i]) <= 5 )
//                    {
//                        addRemovePt = false;
//                        continue;
//                    }
//                }

//                if( !status[i] )
//                    continue;

//                points[1][k++] = points[1][i];
//                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
//            }
//            points[1].resize(k);
//        }

//        if( addRemovePt && points[1].size() < (size_t)MAX_COUNT )
//        {
//            vector<Point2f> tmp;
//            tmp.push_back(point);
//            cornerSubPix( gray, tmp, winSize, Size(-1,-1), termcrit);
//            points[1].push_back(tmp[0]);
//            addRemovePt = false;
//        }

//        needToInit = false;
//        imshow("LK Demo", image);

//        char c = (char)waitKey(10);
//        if( c == 27 )
//            break;
//        switch( c )
//        {
//        case 'r':
//            needToInit = true;
//            break;
//        case 'c':
//            points[0].clear();
//            points[1].clear();
//            break;
//        case 'n':
//            nightMode = !nightMode;
//            break;
//        }

//        std::swap(points[1], points[0]);
//        cv::swap(prevGray, gray);
//    }

//    return 0;
//}
/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

//// Module "core"
//#include <opencv2/core/core.hpp>

//// Module "highgui"
//#include <opencv2/highgui/highgui.hpp>

//// Module "imgproc"
//#include <opencv2/imgproc/imgproc.hpp>

//// Module "video"
//#include <opencv2/video/video.hpp>
//#include "opencv2/video/tracking.hpp"

//#include "opencv2/videoio/videoio.hpp"
//#include <iostream>
//#include <vector>
//using namespace std;
//using namespace cv;
// int main(){
//// >>>>> Color to be tracked
//#define MIN_H_BLUE 200
//#define MAX_H_BLUE 300
//int stateSize = 6;
//  int measSize = 4;
//  int contrSize = 0;

//  unsigned int type = CV_32F;
//  cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

//  cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//  cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//  //cv::Mat procNoise(stateSize, 1, type)
//  // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

//  // Transition State Matrix A
//  // Note: set dT at each processing step!
//  // [ 1 0 dT 0  0 0 ]
//  // [ 0 1 0  dT 0 0 ]
//  // [ 0 0 1  0  0 0 ]
//  // [ 0 0 0  1  0 0 ]
//  // [ 0 0 0  0  1 0 ]
//  // [ 0 0 0  0  0 1 ]
//  cv::setIdentity(kf.transitionMatrix);

//  // Measure Matrix H
//  // [ 1 0 0 0 0 0 ]
//  // [ 0 1 0 0 0 0 ]
//  // [ 0 0 0 0 1 0 ]
//  // [ 0 0 0 0 0 1 ]
//  kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
//  kf.measurementMatrix.at<float>(0) = 1.0f;
//  kf.measurementMatrix.at<float>(7) = 1.0f;
//  kf.measurementMatrix.at<float>(16) = 1.0f;
//  kf.measurementMatrix.at<float>(23) = 1.0f;

//  // Process Noise Covariance Matrix Q
//  // [ Ex 0  0    0 0    0 ]
//  // [ 0  Ey 0    0 0    0 ]
//  // [ 0  0  Ev_x 0 0    0 ]
//  // [ 0  0  0    1 Ev_y 0 ]
//  // [ 0  0  0    0 1    Ew ]
//  // [ 0  0  0    0 0    Eh ]
//  //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
//  kf.processNoiseCov.at<float>(0) = 1e-2;
//  kf.processNoiseCov.at<float>(7) = 1e-2;
//  kf.processNoiseCov.at<float>(14) = 2.0f;
//  kf.processNoiseCov.at<float>(21) = 1.0f;
//  kf.processNoiseCov.at<float>(28) = 1e-2;
//  kf.processNoiseCov.at<float>(35) = 1e-2;

//  // Measures Noise Covariance Matrix R
//  cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
//  // <<<< Kalman Filter
//  // Camera Index
//  int idx = 0;
//  // Camera Capture
// VideoCapture cap;      // >>>>> Camera Settings
//  if (!cap.open(idx))
//  {
//     cout << "Webcam not connected.\n" << "Please verify\n";
//     return EXIT_FAILURE;
//  }

//  cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
//  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
//  // <<<<< Camera Settings

//  cout << "\nHit 'q' to exit...\n";      char ch = 0;      double ticks = 0;    bool found = false;      int notFoundCount = 0;            // >>>>> Main loop
//  while (ch != 'q' || ch != 'Q')
//  {
//     double precTick = ticks;
//     ticks = (double) cv::getTickCount();

//     double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

//     // Frame acquisition
//     Mat frame;
//     cap >> frame;

//     cv::Mat res;
//     frame.copyTo( res );

//     if (found)
//     {
//        // >>>> Matrix A
//        kf.transitionMatrix.at<float>(2) = dT;
//        kf.transitionMatrix.at<float>(9) = dT;
//        // <<<< Matrix A

//        cout << "dT:" << endl << dT << endl;

//        state = kf.predict();
//        cout << "State post:" << endl << state << endl;
//        cv::Rect predRect;
//        predRect.width = state.at<float>(4);
//        predRect.height = state.at<float>(5);
//        predRect.x = state.at<float>(0) - predRect.width / 2;
//        predRect.y = state.at<float>(1) - predRect.height / 2;
//        cv::Point center;
//        center.x = state.at<float>(0);
//        center.y = state.at<float>(1);
//        cv::circle(res, center, 2, CV_RGB(255,0,0), -1);
//        cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
//     }         // >>>>> Noise smoothing
//     cv::Mat blur;
//     cv::GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
//     // <<<<< Noise smoothing         // >>>>> HSV conversion
//     cv::Mat frmHsv;
//     cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
//     // <<<<< HSV conversion         // >>>>> Color Thresholding
//     // Note: change parameters for different colors
//     cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
//     cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),
//           cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
//     // <<<<< Color Thresholding         // >>>>> Improving the result
//     cv::erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
//     cv::dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 2);
//     // <<<<< Improving the result         // Thresholding viewing
//     cv::imshow("Threshold", rangeRes);         // >>>>> Contours detection
//     vector contours;
//     cv::findContours(rangeRes, contours, CV_RETR_EXTERNAL,
//           CV_CHAIN_APPROX_NONE);
//     // <<<<< Contours detection         // >>>>> Filtering
//     vector balls;
//     vector ballsBox;
//     for (size_t i = 0; i < contours.size(); i++)       {          cv::Rect bBox;
//         bBox = cv::boundingRect(contours[i]);
//         float ratio = (float) bBox.width / (float) bBox.height;
//         if (ratio > 1.0f)
//           ratio = 1.0f / ratio;

//        // Searching for a bBox almost square
//        if (ratio > 0.75 && bBox.area() >= 400)
//        {
//           balls.push_back(contours[i]);
//           ballsBox.push_back(bBox);
//        }
//     }
//     // <<<<< Filtering

//     cout << "Balls found:" << ballsBox.size() << endl;         // >>>>> Detection result
//     for (size_t i = 0; i < balls.size(); i++)
//     {
//        cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
//        cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

//        cv::Point center;
//        center.x = ballsBox[i].x + ballsBox[i].width / 2;
//        center.y = ballsBox[i].y + ballsBox[i].height / 2;
//        cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

//        stringstream sstr;
//        sstr << "(" << center.x << "," << center.y << ")";
//        cv::putText(res, sstr.str(),
//              cv::Point(center.x + 3, center.y - 3),
//              cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
//     }
//     // <<<<< Detection result         // >>>>> Kalman Update
//     if (balls.size() == 0)
//     {
//        notFoundCount++;
//        cout << "notFoundCount:" << notFoundCount << endl;          if( notFoundCount >= 10 )
//        {
//           found = false;
//        }
//        else
//           kf.statePost = state;
//     }
//     else
//     {
//        notFoundCount = 0;

//        meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
//        meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
//        meas.at<float>(2) = (float)ballsBox[0].width;
//        meas.at<float>(3) = (float)ballsBox[0].height;

//        if (!found) // First detection!
//        {
//           // >>>> Initialization
//           kf.errorCovPre.at<float>(0) = 1; // px
//           kf.errorCovPre.at<float>(7) = 1; // px
//           kf.errorCovPre.at<float>(14) = 1;
//           kf.errorCovPre.at<float>(21) = 1;
//           kf.errorCovPre.at<float>(28) = 1; // px
//           kf.errorCovPre.at<float>(35) = 1; // px

//           state.at<float>(0) = meas.at<float>(0);
//           state.at<float>(1) = meas.at<float>(1);
//           state.at<float>(2) = 0;
//           state.at<float>(3) = 0;
//           state.at<float>(4) = meas.at<float>(2);
//           state.at<float>(5) = meas.at<float>(3);
//           // <<<< Initialization

//           found = true;
//        }
//        else
//           kf.correct(meas); // Kalman Correction

//        cout << "Measure matrix:" << endl << meas << endl;
//     }
//     // <<<<< Kalman Update

//     // Final result
//     cv::imshow("Risultato finale", res);

//     // User key
//     ch = cv::waitKey(10);
//  }
//  // <<<<< Main loop

//  return EXIT_SUCCESS;
//}
/******************************************
 * OpenCV Tutorial: Ball Tracking using   *
 * Kalman Filter                          *
 ******************************************/

// Module "core"
#include <opencv2/core/core.hpp>

// Module "highgui"
#include <opencv2/highgui/highgui.hpp>

// Module "imgproc"
#include <opencv2/imgproc/imgproc.hpp>

// Module "video"
#include <opencv2/video/video.hpp>
#include "opencv2/videoio.hpp"
#include<opencv2/video/tracking.hpp>



// Output
//#include <iostream>
//#include <stdio.h>
// Vector
//#include <vector>
//#include <string>
//#include "singlekalmanfilter.h"
//using namespace std;
//using namespace cv;
// >>>>> Color to be tracked
//#define MIN_H_BLUE 200
//#define MAX_H_BLUE 300
// <<<<< Color to be tracked


//int main()
//{
//    // Camera frame
//    cv::Mat frame;
//    cv::Mat fgMaskMOG;
//    cv::Ptr<cv::BackgroundSubtractor> pMOG;
//    // >>>> Kalman Filter
//    int stateSize = 6;
//    int measSize = 4;
//    int contrSize = 0;


//    SingleKalmanFilter kf;

//    unsigned int type = CV_32F;
//    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
//    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
//    //cv::Mat procNoise(stateSize, 1, type)
//    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]



//    // Camera Index
//    int idx = 0;

//    // Camera Capture
//   cv::VideoCapture cap("F:/test3.avi");
//     // VideoCapture cap;
//     // cap.open(0);
//    // >>>>> Camera Setting

//    if(!cap.isOpened())
//    {
//        cout <<"Video open error\n Error";
//        return -1;

//    }
//    //    {
//    //        cout << "Webcam not connected.\n" << "Please verify\n";
//    //        return EXIT_FAILURE;
//    //    }

//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1024);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 768);
//    // <<<<< Camera Settings

//    cout << "\nHit 'q' to exit...\n";

//    char ch = 0;

//    double ticks = 0;
//    bool found = false;

//    int notFoundCount = 0;

//    pMOG = createBackgroundSubtractorMOG2() ; //MOG approach

//    // >>>>> Main loop
//    while (ch != 'q' && ch != 'Q')
//    {
//        double precTick = ticks;
//        ticks = (double) cv::getTickCount();

//        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

//        // Frame acquisition
//        cap >> frame;

//        cv::Mat res;
//        frame.copyTo( res );

//        if (found)
//        {
//            // >>>> Matrix A
//            kf.setTransitionMatrix(2,dT);
//            kf.setTransitionMatrix(9,dT);
//            // <<<< Matrix A

//            cout << "dT:" << endl << dT << endl;

//            state = kf.predict();
//            cout << "State post:" << endl << state << endl;

//            cv::Rect predRect;
//            predRect.width = state.at<float>(4);
//            predRect.height = state.at<float>(5);
//            predRect.x = state.at<float>(0) - predRect.width / 2;
//            predRect.y = state.at<float>(1) - predRect.height / 2;

//            cv::Point center;
//            center.x = state.at<float>(0);
//            center.y = state.at<float>(1);
//            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

//            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
//        }

//        // >>>>> Noise smoothing
//        cv::Mat blur;
//        cv::GaussianBlur(frame, blur, cv::Size(5, 5), 0, 0);
//        // <<<<< Noise smoothing

//        // >>>>> HSV conversion
//        cv::Mat frmHsv;
//        cv::cvtColor(blur, frmHsv, CV_BGR2HSV);
//        // <<<<< HSV conversion


//        // >>>>> Color Thresholding
//        // Note: change parameters for different colors
//        cv::Mat rangeRes = cv::Mat::zeros(frame.size(), CV_8UC1);
//        cv::inRange(frmHsv, cv::Scalar(MIN_H_BLUE / 2, 100, 80),cv::Scalar(MAX_H_BLUE / 2, 255, 255), rangeRes);
//        // <<<<< Color Thresholding

//        // >>>>> Improving the result

//        // <<<<< Improving the result
//        pMOG->apply(frame,fgMaskMOG);

//        int  morph_size = 2;
//        Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );

//        for (int i = 0;i<3;i++)
//        {
//         morphologyEx( fgMaskMOG, fgMaskMOG, MORPH_OPEN, element, Point(-1,-1), i );
//        }
//        // Thresholding viewing

//        cv::imshow("MOG", fgMaskMOG);
//        //cv::imshow("Threshold", frame);

//        //Create parameters for Harris corner

//        vector<Point2f> features;

//        Mat dst, dst_norm, dst_norm_scaled;
//        dst = Mat::zeros(fgMaskMOG.size(), CV_32FC1);
//        int blockSize = 7;
//      int apertureSize = 5;
//      double k = 0.05;
//         // cornerHarris(fgMaskMOG, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
//        Canny(fgMaskMOG,dst,7,1);

//        goodFeaturesToTrack(dst, features,1500,0.05,20,noArray(),3,false);
//        // Normalizing
//        //        normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
//        //        convertScaleAbs( dst_norm, dst_norm_scaled );

//        //        // Drawing a circle around corners
//        //        vector<Point> box;
//        //        for( int j = 0; j < dst_norm.rows ; j++ )
//        //        {
//        //            for( int i = 0; i < dst_norm.cols; i++ )
//        //            {
//        //                if( (int) dst_norm.at<float>(j,i) > 220 )
//        //                {
//        //                    circle( dst_norm_scaled, Point( i, j ), 8,  Scalar(120), 2, 8, 0 )ZZ;

//        //                }

//        //            }
//        //        }

//        for( uint j = 0; j < features.size() ; j++ )
//        {
//            circle(dst,features.at(j),8,Scalar(120),2);
//        }


//      imshow("Corners Harris",fgMaskMOG);


//      imshow("canny", dst);
//        // >>>>> Contours detection
//        vector<vector<cv::Point> > contours;
//        cv::findContours(dst, contours, CV_RETR_LIST,
//                         CV_CHAIN_APPROX_NONE);
//        // <<<<< Contours detection


//        // >>>>> Filtering
//        vector<vector<cv::Point> > balls;
//        vector<cv::Rect> ballsBox;
//        for (size_t i = 0; i < contours.size(); i++)
//        {
//            cv::Rect bBox;
//            vector<Point> contour;
//            approxPolyDP(contours[i],contour,0.05,true);
//            bBox = cv::boundingRect(contour);

//            float ratio = (float) bBox.width / (float) bBox.height;
//            if (ratio > 1.0f)
//                ratio = 1.0f / ratio;

//            // Searching for a bBox almost square
//            if (ratio > 0.75 && bBox.area() >= 400)
//            {
//                balls.push_back(contours[i]);
//                ballsBox.push_back(bBox);
//            }
//        }
//        // <<<<< Filtering



//        cout << "Balls found:" << ballsBox.size() << endl;

//        // >>>>> Detection result
//        for (size_t i = 0; i < balls.size(); i++)
//        {
//            //cv::drawContours(res, balls, i, CV_RGB(20,150,20), 1);
//            cv::rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

//            cv::Point center;
//            center.x = ballsBox[i].x + ballsBox[i].width / 2;
//            center.y = ballsBox[i].y + ballsBox[i].height / 2;
//            cv::circle(res, center, 2, CV_RGB(20,150,20), -1);

//            stringstream sstr;
//            sstr << "(" << center.x << "," << center.y << ")";
//            cv::putText(res, sstr.str(),
//                        cv::Point(center.x + 3, center.y - 3),
//                        cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
//        }
//        // <<<<< Detection result

//        // >>>>> Kalman Update
//        if (balls.size() == 0)
//        {
//            notFoundCount++;
//            cout << "notFoundCount:" << notFoundCount << endl;
//            if( notFoundCount >= 100 )
//            {
//                found = false;
//            }
//            /*else
//                kf.statePost = state;*/
//        }
//        else
//        {
//            notFoundCount = 0;

//            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
//            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
//            meas.at<float>(2) = (float)ballsBox[0].width;
//            meas.at<float>(3) = (float)ballsBox[0].height;

//            if (!found) // First detection!
//            {
//                // >>>> Initialization
//                kf.setErrorCov(0,1);
//                kf.setErrorCov(7,1);
//                kf.setErrorCov(14,1);
//                kf.setErrorCov(28,1);
//                kf.setErrorCov(35,1);


//                state.at<float>(0) = meas.at<float>(0);
//                state.at<float>(1) = meas.at<float>(1);
//                state.at<float>(2) = 0;
//                state.at<float>(3) = 0;
//                state.at<float>(4) = meas.at<float>(2);
//                state.at<float>(5) = meas.at<float>(3);
//                // <<<< Initialization

//                found = true;
//            }
//            else
//                kf.correct(meas); // Kalman Correction

//            cout << "Measure matrix:" << endl << meas << endl;
//        }
//        // <<<<< Kalman Update

//        // Final result
//        cv::imshow("Tracking", res);

//        // User key
//        ch = cv::waitKey(1);
//    }
//    // <<<<< Main loop

//    return EXIT_SUCCESS;
//}


#include "reader.h"
#include "detection.h"
#include <QApplication>

int main(int argc,char *argv[]){
    QApplication app(argc, argv);
    Reader reader;
    cv::Point size = reader.openFromFile();
    Detection detect;
    detect.calcMaxDistance(size.x,size.y);
        char ch = 0;
    while(ch!='q')
    {


    detect.detectObjects(reader.getFrame());

    ch = cv::waitKey(1);
        }



    return app.exec();




}
