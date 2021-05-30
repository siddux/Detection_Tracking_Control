//
// Created by Eduard  Rius Martínez on 27/11/2020.
//

#include <iostream>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
using namespace cv;
using namespace std;
int main(int argc, char **argv)
{

    /*VideoCapture capture(filename);
    if (!capture.isOpened()){
        //error in opening the video input
        cerr << "Unable to open file!" << endl;
        return 0;
    }*/
    Mat frame, roi, hsv_roi, mask;
    VideoCapture capture;
    capture.open(0, cv::CAP_V4L2);
    capture.set(cv::CAP_PROP_FPS, 30);
    // take first frame of the video
    capture >> frame;
    // setup initial location of window
    //Rect track_window(278, 246, 98, 237); // simply hardcoded the values
    Rect track_window(50, 50, 10, 10);
    // set up the ROI for tracking
    roi = frame(track_window); //Provided by YOLOm

    cvtColor(roi, hsv_roi, COLOR_BGR2HSV);
    inRange(hsv_roi, Scalar(0, 60, 32), Scalar(180, 255, 255), mask); //Threshold
    float range_[] = {0, 180};
    const float* range[] = {range_};
    Mat roi_hist;
    int histSize[] = {180};
    int channels[] = {0};
    calcHist(&hsv_roi, 1, channels, mask, roi_hist, 1, histSize, range);
    normalize(roi_hist, roi_hist, 0, 255, NORM_MINMAX);
    // Setup the termination criteria, either 10 iteration or move by atleast 1 pt
    TermCriteria term_crit(TermCriteria::EPS | TermCriteria::COUNT, 10, 1);
    while(true){
        Mat hsv, dst;
        capture >> frame;
        if (frame.empty())
            break;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        calcBackProject(&hsv, 1, channels, roi_hist, dst, range);
        // apply camshift to get the new location
        RotatedRect rot_rect = CamShift(dst, track_window, term_crit);
        // Draw it on image
        Point2f points[4];
        rot_rect.points(points);
        for (int i = 0; i < 4; i++)
            line(frame, points[i], points[(i+1)%4], 255, 2);
        imshow("img2", frame);
        int keyboard = waitKey(30);
        if (keyboard == 'q' || keyboard == 27)
            break;
    }
}

