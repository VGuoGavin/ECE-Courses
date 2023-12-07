//
// Created by Shiping Guo on 29/9/23.
//

#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class EventCamera{
public:
    Mat frame_current, frame_last, interpolated_frame;
    VideoCapture* camera;
    Mat WhiteBackground;

    EventCamera();
    ~EventCamera();
    vector<int**> getDiff();
    Mat logTransform(Mat srcImage, int c);
};