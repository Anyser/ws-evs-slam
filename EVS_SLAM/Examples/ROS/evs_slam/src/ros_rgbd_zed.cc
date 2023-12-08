/**
* This file is a modified version of ORB-SLAM2.<https://github.com/raulmur/ORB_SLAM2>
*
* This file is part of EVS_SLAM
*/

#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include<evs_slam/msg_mask.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include"../../../include/System.h"
#include <functional> 

using namespace std;
using namespace cv;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM, bool wod):mpSLAM(pSLAM), wod(wod){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgMask);
    
    ORB_SLAM2::System* mpSLAM;
    bool wod;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc < 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings wod" << endl;        
        ros::shutdown();
        return 1;
    }    

    string voc_dir=argv[1];
    string config_dir=argv[2];
    bool wod = std::string(argv[3]) == "True" ? true : false;

    
    
    ORB_SLAM2::System SLAM(voc_dir,config_dir,ORB_SLAM2::System::RGBD,true);

    ImageGrabber ig(&SLAM, wod);

    ros::NodeHandle nh;


    int buffer_size = 10;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/img/rgb", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/img/depth", 1);
    message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/img/mask", 1);

    ros::topic::waitForMessage<sensor_msgs::Image>("/img/rgb");
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/depth");
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/mask");


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(buffer_size), rgb_sub,depth_sub, mask_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&ig,_1,_2, _3));
    
    ros::spin();

    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD, const sensor_msgs::ImageConstPtr& msgMask)
{

     if(!msgRGB)
    {
        ROS_WARN("Mensaje RGB no recibido!");
        return;
    }
    
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrMask;
    try
    {
        cv_ptrMask = cv_bridge::toCvShare(msgMask, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image, cv_ptrMask->image, cv_ptrRGB->header.stamp.toSec(), wod);
}


