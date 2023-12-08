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

    void GrabSTEREO(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, const sensor_msgs::ImageConstPtr& msgMask);
    
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

    
    
    ORB_SLAM2::System SLAM(voc_dir,config_dir,ORB_SLAM2::System::STEREO,true);

    ImageGrabber ig(&SLAM, wod);

    ros::NodeHandle nh;


    int buffer_size = 10;
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/img/left", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/img/right", 1);
    message_filters::Subscriber<sensor_msgs::Image> mask_sub(nh, "/img/mask", 1);

    ros::topic::waitForMessage<sensor_msgs::Image>("/img/left");
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/right");
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/mask");


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(buffer_size), left_sub,right_sub, mask_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabSTEREO,&ig,_1,_2, _3));
    
    ros::spin();

    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_STEREO.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabSTEREO(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight, const sensor_msgs::ImageConstPtr& msgMask)
{

    if(!msgLeft)
    {
        ROS_WARN("error in img left!");
        return;
    }
    
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(!msgRight)
    {
        ROS_WARN("error in img right!");
        return;
    }
    
    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
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

    mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image, cv_ptrMask->image, cv_ptrLeft->header.stamp.toSec(), wod);
}


