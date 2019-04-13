#define CMT_DISPLAY

#include "CMT.h"
#include "gui.h"

#include <string>
#include <ctime>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaoptflow.hpp>

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
float kf_rect[4][2];

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
    pol.points[0].z = cmt.points_active.size();
    pub.publish(pol);
}

void KFCallback(const Polygon::ConstPtr& p){
    for(int i=2;i<=5;i++){
        kf_rect[i-2][0] = p->points[i].x;
        kf_rect[i-2][1] = p->points[i].y;
    }
}

void drawKFRect(Mat& im){
    for (int i = 0; i < 4; i++)
        line(im, Point2f(kf_rect[i][0],kf_rect[i][1]), Point2f(kf_rect[(i+1)%4][0],kf_rect[(i+1)%4][1]), Scalar(0,255,255));
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
    cap.open(0); // opens the default camera
    //cap.open("/home/dji/Desktop/slow.flv");
    if(!cap.isOpened()){
        ROS_ERROR("Unable to open video input");
        return -1;
    }

    /* Set desired capture properties(320x240@120fps,MJPG) */
    //cap.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,320.0);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,240.0);
    cap.set(cv::CAP_PROP_FPS,120.0);

    /* Initialize the calibration matrix */
    Mat cameraMatrix = Mat::eye(3,3,CV_64F);
    cameraMatrix.at<double>(0,0) = 4.8025052410787254e+02;
    cameraMatrix.at<double>(1,1) = 4.7946924964527756e+02;
    cameraMatrix.at<double>(0,2) = 2.9451792432725512e+02;
    cameraMatrix.at<double>(1,2) = 2.7471736848806802e+02;

    /* Initialize the distortion coefficient */
    Mat distCoeff = Mat::zeros(4,1,CV_64F);
    distCoeff.at<double>(0,0) = 1.1577520980447170e-01;
    distCoeff.at<double>(1,0) = -1.4641950413598775e-01;
    distCoeff.at<double>(2,0) = -4.4117726647507601e-04;
    distCoeff.at<double>(3,0) = -2.8091325383948956e-04;

    /* Show preview until a key is pressed */
    Mat preview, preview_tmp;
    char k;
    while(true){
        cap >> preview_tmp;
        cv::undistort(preview_tmp,preview,cameraMatrix,distCoeff);
        screenLog(preview,"Press any key to start specifying the drone to follow");
        imshow(WIN_NAME,preview);
        k = waitKey(10);
        if(k==' ')
            break;
    }

    /* Get the initial bounding box */
    Mat im0,im0_tmp;
    cap >> im0_tmp;//TODO: verify the shape 8UC3/8UC1?
    cv::undistort(im0_tmp,im0,cameraMatrix,distCoeff);
    //resize(im0,im0,Size(),scale,scale);
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

    /* Subscribe to Kalman filter output */
    ros::Subscriber sub = n.subscribe("/kalman_output",10,KFCallback);
    clock_t begin,end;
    int time_elapsed;
    Mat im,im_tmp,im_prev=im0_gray;
    GpuMat im_gpu,im_gpu_prev;
    im_gpu_prev.upload(im_prev);

    /* Main loop */
    while(ros::ok()){
        /* Read and resize the input frame */
        cap >> im_tmp;
        cv::undistort(im_tmp,im,cameraMatrix,distCoeff);
        //resize(im,im,Size(),scale,scale);
        cvtColor(im,im,CV_BGR2GRAY);

        /* Upload data to GPU */
        im_gpu.upload(im);
        /* Process the frame with CMT and log the time */
        // TODO: test whether allocation every loop or clone is faster
        begin = clock();
        cmt.processFrame(im_gpu,im_gpu_prev);
        end = clock();
        time_elapsed = (end-begin)*1.0/CLOCKS_PER_SEC*1000;
        ROS_INFO("Time: %dms, active features: %lu",time_elapsed,cmt.points_active.size());

#ifdef CMT_DISPLAY
        /* Display the image */
        drawKFRect(im);
        display(im,cmt);
#endif
        sendtoROS(pub,cmt);
        im_gpu.swap(im_gpu_prev);
        ros::spinOnce();
    }
    
    return 0;
}
