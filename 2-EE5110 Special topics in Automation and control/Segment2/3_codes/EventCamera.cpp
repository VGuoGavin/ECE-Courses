//
// Created by Shiping Guo on 20/9/23.
//
#include <opencv2/opencv.hpp>
#include "EventCamera.h"
#include <cmath>

using namespace cv;
using namespace std;

using namespace cv;

EventCamera::EventCamera(){

}
EventCamera::~EventCamera(){

}
vector<int**> EventCamera::getDiff() {
        frame_last = frame_current.clone();
        camera->read(frame_current);

        //test input image
        //frame_current = imread("/Users/shipingguo/CLionProjects/EE5110_CA2/Videos/960fps.png");

        /* frame interpolate */
        /*
        int interpolated_frame_num = 1;
        float alpha = 0.5;
        addWeighted(frame_last, 1 - alpha, frame_current, alpha, 0, interpolated_frame);
        */


        /* Log transform */
        //frame_current = logTransform(frame_current, 6 );
        //imshow("Without noise", frame_current);

        /*  Add noise  */
        /*
        Mat gauss_noise=Mat::zeros(frame_current.rows,frame_current.cols,frame_current.type());
        RNG rng;            //Creat RNG class
        rng.fill(gauss_noise,RNG::NORMAL,10,20);    //Generate three channels image
        frame_current = frame_current+gauss_noise;
        imwrite("/Users/shipingguo/CLionProjects/EE5110_CA2/Videos/With_Noise.jpg", frame_current);
        imshow("With Noise", frame_current);
        waitKey(0);
        */

        int ** temp_mat_B = new int* [frame_current.rows];
        for (int i = 0; i < frame_current.rows; i++) {
            temp_mat_B[i] = new int[frame_current.cols];
        }
        int** temp_mat_G = new int* [frame_current.rows];

        for (int i = 0; i < frame_current.rows; i++) {

            temp_mat_G[i] = new int[frame_current.cols];
        }
        int** temp_mat_R = new int* [frame_current.rows];

        for (int i = 0; i < frame_current.rows; i++) {

            temp_mat_R[i] = new int[frame_current.cols];
        }

        int pixel_B, pixel_G, pixel_R;

        int threshold_value = 10; //Value of the threshold under which the differences between each frames will be negleted

        for (int i = 0; i < frame_current.cols; i++)
        {
            for (int j = 0; j < frame_current.rows; j++)
            {
                pixel_B = (int)frame_current.at<Vec3b>(j, i)[0] - (int)frame_last.at<Vec3b>(j, i)[0];
                pixel_G = (int)frame_current.at<Vec3b>(j, i)[1] - (int)frame_last.at<Vec3b>(j, i)[1];
                pixel_R = (int)frame_current.at<Vec3b>(j, i)[2] - (int)frame_last.at<Vec3b>(j, i)[2];

                if (abs(pixel_B) < threshold_value)
                {
                    temp_mat_B[j][i] = 255;
                }
                else
                {
                    temp_mat_B[j][i] = pixel_B;
                }

                if (abs(pixel_G) < threshold_value)
                {
                    temp_mat_G[j][i] = 255;
                }
                else
                {
                    temp_mat_G[j][i] = pixel_G;
                }

                if (abs(pixel_R) < threshold_value)
                {
                    temp_mat_R[j][i] = 255;
                }
                else
                {
                    temp_mat_R[j][i] = pixel_R;
                }

            }
        }
        vector <int**> mat_BGR{ temp_mat_B ,temp_mat_G ,temp_mat_R };
        return mat_BGR;
}

// Do the log transformation
Mat EventCamera::logTransform(cv::Mat srcImage, int c)
{
    //WhiteBackground = srcImage.rows, srcImage.cols, CV_8UC1, Scalar(255,255,255);
    //imshow("srcImage", srcImage);
    //Mat srcImage = frame_current.clone();
    Mat temp_mat_RGB = srcImage.clone();
    Mat temp_RGB = srcImage.clone();
    int pixcel_B, pixcel_G, pixcel_R;
    float max_value[3]={0,0,0};
    float min_value[3]={0,0,0};

    for(int n =0; n<3; n++){
        for (int i = 0; i < srcImage.cols; i++){
            for (int j = 0; j < srcImage.rows; j++){
                if (max_value[n] < 0.5* log(int(srcImage.at<Vec3b>(j, i)[n])+1)){
                    max_value[n] =  0.5*log(int(srcImage.at<Vec3b>(j, i)[n])+1);
                }
                if (min_value[n] >  0.5*log(int(srcImage.at<Vec3b>(j, i)[n])+1)){
                    min_value[n] =  0.5*log(int(srcImage.at<Vec3b>(j, i)[n])+1);
                }
            }
        }
    }

    for(int n=0; n<3; n++){
        float scale = 255/(max_value[n] - min_value[n]);
        for (int i = 0; i < srcImage.cols; i++){
            for (int j = 0; j < srcImage.rows; j++){
                //temp_mat_RGB.at<Vec3b>(j, i)[n] = log(int(srcImage.at<Vec3b>(j, i)[n])+1);
                temp_RGB.at<Vec3b>(j, i)[n] = int(0.5*(log(int(srcImage.at<Vec3b>(j, i)[n])+1)-min_value[n]) * scale);
            }
        }
    }
    //vector <int **> matBGR{ temp_B ,temp_G ,temp_R };
    //imshow("resultImage", temp_RGB);
    //imwrite("/Users/shipingguo/CLionProjects/opencvtest/Log_result.jpg", temp_RGB);
    //waitKey(0);
    return temp_RGB;
}
