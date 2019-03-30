#include "CMT.h"
#include "gui.h"

#include <string>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>

using cmt::CMT;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::min_element;
using std::max_element;

const string WIN_NAME = "CMT Node";

void display(Mat im, CMT & cmt)
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    imshow(WIN_NAME, im);
    waitKey(10);
}

int main(int argc, char *argv[]){
    CMT cmt;
    Rect rect; 
    namedWindow(WIN_NAME);

    VideoCapture cap;
    cap.open(0); // opens the default camera
    if(!cap.isOpened()){
        ROS_ERROR("Unable to open video input");
        return -1;
    }
    
    // Show preview until a key is pressed
    Mat preview;
    char k;
    while(true){
        cap >> preview;
        screenLog(preview,"Press any key to start specifying the bounding box");
        imshow(WIN_NAME,preview);
        k = waitKey(10);
        if(k!=-1)
            break;
    }

    // Get the initial bounding box
    Mat im0;
    cap >> im0;
    rect = getRect(im0,WIN_NAME);
    ROS_INFO("Using bounding box (%d,%d,%d,%d)",rect.x,rect.y,rect.width,rect.height);
    
    // Initialize CMT with grayscale image
    Mat im0_gray;
    cvtColor(im0,im0_gray,CV_BGR2GRAY);
    cmt.initialize(im0_gray,rect);

    // Main loop
    
    
    while(true){
        Mat im;
        cap >> im;
        Mat im_gray;
        cvtColor(im,im_gray,CV_BGR2GRAY);

        // Process the frame with CMT
        cmt.processFrame(im_gray);
        ROS_INFO("Active features: %lu",cmt.points_active.size());

        // Display the image
        display(im,cmt);
    }

    return 0;
}
