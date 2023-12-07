//
// Created by Shiping Guo on 20/9/23.
//

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
#include "EventCamera.h"

int main(int, char**)
{
    Mat current_frame,last_frame;
    Mat Event_difference, Event_difference2;
    Mat Event_difference_init, Event_difference_init2, BlackScreen, difference, Event;

    /*  Use input videos  */
    //VideoCapture cap("/Users/shipingguo/CLionProjects/opencvtest/Slow Motion.mp4");
    //VideoCapture cap("/Users/shipingguo/CLionProjects/EE5110_CA2/Videos/Walk.mp4");


    /*  Use lap-top camera  */
    VideoCapture cap;
    int deviceID = 0;
    int apiID = cv::CAP_ANY;
    //Preffered camera capture size
    cap.set(3, 512);
    cap.set(4, 288);
    cap.open(deviceID, apiID);
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cap.read(current_frame);       // read one frame
    Mat BlackScreen_temp(current_frame.rows, current_frame.cols, CV_8UC1, Scalar(0,0,0));
    Mat in[] = { BlackScreen_temp, BlackScreen_temp, BlackScreen_temp };
    merge(in, 3, BlackScreen);

    EventCamera ec;
    ec.camera = &cap;
    ec.frame_current = BlackScreen.clone();
    ec.frame_last = BlackScreen.clone();

    difference,Event = BlackScreen.clone();
    Event_difference = BlackScreen.clone();

    vector<int**> mats;
    // Define the event
    for (;;)
    {
        mats = ec.getDiff(); // store the difference

        difference = Event.clone();

        for (int i = 0; i < difference.cols; i++)
        {
            for (int j = 0; j < difference.rows; j++)
            {
                if ((int)difference.at<Vec3b>(j, i)[0] + (int)mats[0][j][i] > 255)
                {
                    Event.at<Vec3b>(j, i)[0] = 255;
                }
                else if ((int)difference.at<Vec3b>(j, i)[0] + (int)mats[0][j][i] < 0)
                {
                    Event.at<Vec3b>(j, i)[0] = 0;
                }
                else
                {
                    Event.at<Vec3b>(j, i)[0] = (int)difference.at<Vec3b>(j, i)[0] + (int)mats[0][j][i];
                }

                if ((int)difference.at<Vec3b>(j, i)[1] + (int)mats[1][j][i] > 255)
                {
                    Event.at<Vec3b>(j, i)[1] = 255;
                }
                else if ((int)difference.at<Vec3b>(j, i)[1] + (int)mats[1][j][i] < 0)
                {
                    Event.at<Vec3b>(j, i)[1] = 0;
                }
                else
                {
                    Event.at<Vec3b>(j, i)[1] = (int)difference.at<Vec3b>(j, i)[1] + (int)mats[1][j][i];
                }

                if ((int)difference.at<Vec3b>(j, i)[2] + (int)mats[2][j][i] > 255)
                {
                    Event.at<Vec3b>(j, i)[2] = 255;
                }
                else if ((int)difference.at<Vec3b>(j, i)[2] + (int)mats[2][j][i] < 0)
                {
                    Event.at<Vec3b>(j, i)[2] = 0;
                }
                else
                {
                    Event.at<Vec3b>(j, i)[2] = (int)difference.at<Vec3b>(j, i)[2] + (int)mats[2][j][i];
                }

                Event_difference.at<Vec3b>(j, i)[0] = (int)mats[0][j][i];
                Event_difference.at<Vec3b>(j, i)[1] = (int)mats[1][j][i];
                Event_difference.at<Vec3b>(j, i)[2] = (int)mats[2][j][i];
            }
        }
        Mat blur;   // add noise or do the filter
        //cv::GaussianBlur(Event, blur, Size(15, 15),0);

        imshow("Just Event", Event);
        imshow("Frame + Event_difference", Event_difference);

        waitKey(10);

        //if (waitKey(1) >= 0)
        //    break;
    }

    // store output
    //VideoWriter video("output_event.mp4",CV_FOURCC('M','J','P','G'),10, Size(frame_width,frame_height));
    return 0;
}