#include <iostream>
#include <sstream>
#include <cstdio>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
//#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "tf/transform_broadcaster.h"

using namespace std;
using namespace cv;

ros::NodeHandle *node;
//image_transport::Subscriber sub;
//ros::Subscriber sub;
ros::Subscriber *sub_image_source;
//image_transport::Publisher faces_highlighted;
ros::Publisher faces_highlighted;

string nodename;

CascadeClassifier face_cascade;

void received_frame(const sensor_msgs::Image::ConstPtr &msg) {
    static tf::TransformBroadcaster br;
    static char buffer[1024];
    int x, y;
    int i;
    cv_bridge::CvImagePtr work;

    work = cv_bridge::toCvCopy(msg);
/*
    circle(work->image, Point(20, 20), 10, CV_RGB(255, 0, 0));
    cvtColor(work->image, work->image, CV_RGB2HSV);
    for(y = 0; y < work->image.rows; y++) {
        for(x = 0; x < work->image.cols; x++) {
            work->image.at<Vec3b>(y, x)[1] = 248;
            work->image.at<Vec3b>(y, x)[2] = 248;
        };
    };

    cvtColor(work->image, work->image, CV_HSV2RGB);
*/


    vector<Rect> faces;
    Mat frame_gray;
    cvtColor(work->image, frame_gray, CV_RGB2GRAY);
    equalizeHist(frame_gray, frame_gray);
    face_cascade.detectMultiScale(frame_gray, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, Size(30, 30));
    for(i = 0; i < (int)faces.size(); i++) {
        Point center(faces[i].x + faces[i].width / 2, faces[i].y + faces[i].height / 2);
        double distance = 100 / (double)faces[i].width;
        double fovx = M_PI / 2;
        double fovy = M_PI / 2;
        double yaw = (faces[i].x + faces[i].width / 2) * fovx / work->image.cols - fovx / 2;
        double pitch = -(faces[i].y + faces[i].height / 2) * fovy / work->image.rows + fovy / 2;
        ellipse(work->image, center, Size(faces[i].width / 2, faces[i].height / 2), 0, 0, 360, Scalar(255, 0, 255), 4);
        snprintf(buffer, 1024, "(%4.3f, %+4.3f, %+4.3f)", distance, yaw, pitch);
        putText(work->image, buffer, center, 0, 0.5, Scalar(0, 255, 0));

        tf::Transform transform;
        double transx, transy, transz;
        transx = distance * sin(yaw);
        transy = distance * sin(pitch);
        transz = sqrt(distance * distance - transx * transx - transy * transy);
        transform.setOrigin(tf::Vector3(transx, transy, transz));
        printf("transx: %f\ttransy: %f\ttransz: %f\n", transx, transy, transz);
        transform.setRotation(tf::Quaternion(M_PI, 0, 0));
        snprintf(buffer, 1024, "/face_%d", i);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/camera", buffer));
    };

    faces_highlighted.publish(work->toImageMsg());
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_detector", ros::init_options::AnonymousName);

    node = new ros::NodeHandle();
    nodename = ros::this_node::getName();

    //image_transport::ImageTransport it(*node);
    //sub = it.subscribe("/usb_cam/image_raw", 1, recieved_frame);
    //sub = it.subscribe("/usb_cam/image_raw/compressed", 1, recieved_frame);
    //sub = node->subscribe("/usb_cam/image_raw", 1, recieved_frame);
    faces_highlighted = node->advertise<sensor_msgs::Image>(nodename + "/faces_highlighted", 1);
    //faces_highlighted = it.advertise("/opencv_test/faces_highlighted", 1);

    if(! face_cascade.load("/usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml")) {
    //if(! face_cascade.load("haarcascade_frontalface_alt.xml")) {
    //if(! face_cascade.load("haarcascade_upperbody.xml")) {
        cout << "Could not load face file" << endl;
        return -1;
    };

    if(! node->hasParam(nodename + "/image_source"))
        node->setParam(nodename + "/image_source", "image");
    node->setParam(nodename + "/image_source__meta/type", "string");
    node->setParam(nodename + "/image_source__meta/defines", "topic");
    node->setParam(nodename + "/image_source__meta/topic_type", "sensor_msgs/Image");

    string image_source, new_image_source;
    ros::Rate r(100);
    while(ros::ok()) {
        if(node->getParam(nodename + "/image_source", new_image_source)) {
            if(new_image_source != image_source) {
                image_source = new_image_source;
                if(sub_image_source) {
                    delete sub_image_source;
                    sub_image_source = NULL;
                };
                sub_image_source = new ros::Subscriber(node->subscribe(image_source, 1, received_frame));
            };
        };
        ros::spinOnce();
        r.sleep();
    };

    node->deleteParam(nodename);
    delete node;

    return 0;
}

