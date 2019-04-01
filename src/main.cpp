#define CMT_DISPLAY

#include "CMT.h"
#include "gui.h"

#include <string>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Polygon.h>

using cmt::CMT;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using cv::Size;
using std::min_element;
using std::max_element;
using ros::Publisher;
using geometry_msgs::Polygon;
using geometry_msgs::Point32;

const string WIN_NAME = "CMT Node";
const float scale = 0.5;

#ifdef CMT_DISPLAY
void display(Mat im, CMT & cmt){
    /* Visualize the output */
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
    waitKey(20);
}
#endif

void sendtoROS(Publisher& pub, CMT& cmt){
    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    
    Polygon pol;
    Point32 p;
    for (int i = 0; i < 4; i++)
    {
        p.x = vertices[i].x;
        p.y = vertices[i].y;
        pol.points.push_back(p);
    }
    pub.publish(pol);
}

int main(int argc, char *argv[]){
    CMT cmt;
    Rect rect; 
    namedWindow(WIN_NAME);
    
    /* Initialize ROS */
    ros::init(argc,argv,"cmt_node");
    ros::NodeHandle n("~");
    Publisher pub = n.advertise<Polygon>("cmt_output",10);

    /* Initialize the input device */
    VideoCapture cap;
    //cap.open(1); // opens the default camera
    cap.open("/home/dji/Desktop/slow.flv");
    if(!cap.isOpened()){
        ROS_ERROR("Unable to open video input");
        return -1;
    }
    
    /* Show preview until a key is pressed */
    Mat preview;
    char k;
    while(true){
        cap >> preview;
        screenLog(preview,"Press any key to start specifying the drone to follow");
        imshow(WIN_NAME,preview);
        k = waitKey(10);
        if(k!=-1)
            break;
    }

    /* Get the initial bounding box */
    Mat im0;
    cap >> im0;
    resize(im0,im0,Size(),scale,scale);
    rect = getRect(im0,WIN_NAME);
    ROS_INFO("Using bounding box (%d,%d,%d,%d)",rect.x,rect.y,rect.width,rect.height);
    
    /* Configure and initialize CMT with grayscale image */
    //cmt.consensus.estimate_scale = false;
    //cmt.consensus.estimate_rotation = true;
    Mat im0_gray;
    cvtColor(im0,im0_gray,CV_BGR2GRAY);
    cmt.initialize(im0_gray,rect);

#ifndef CMT_DISPLAY
    cv::destroyWindow(WIN_NAME);
#endif
    clock_t begin,end;
    int time_elapsed;
    /* Main loop */
    while(ros::ok()){
        /* Read and resize the input frame */
        Mat im;
        Mat im_gray; // Mat performs shallow copy, has to allocate every time
        cap >> im;
        resize(im,im,Size(),scale,scale);
        cvtColor(im,im_gray,CV_BGR2GRAY);

        /* Process the frame with CMT and log the time */
        // TODO: test whether allocation every loop or clone is faster
        begin = clock();
        cmt.processFrame(im_gray);
        end = clock();
        time_elapsed = (end-begin)*1.0/CLOCKS_PER_SEC*1000;
        ROS_INFO("Time: %dms, active features: %lu",time_elapsed,cmt.points_active.size());

#ifdef CMT_DISPLAY
        /* Display the image */
        display(im,cmt);
#endif
        sendtoROS(pub,cmt);
    }

    return 0;
}
