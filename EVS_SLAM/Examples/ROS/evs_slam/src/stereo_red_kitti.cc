#include<iostream>
#include<chrono>
//#include<algorithm>

#include<ros/ros.h>
#include<orb_slam2/msg_mask.h>
//#include<std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include"../../../include/System.h"


using namespace std;
using namespace cv;


class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    
    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,const orb_slam2::msg_mask::ConstPtr& msg);
    
    //vector<uint8_t> gdata_net;
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_red");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1; 
    } 

    string voc_dir=argv[1];
    string config_dir=argv[2];

    ORB_SLAM2::System SLAM(voc_dir,config_dir,ORB_SLAM2::System::STEREO,true);

    ImageGrabber ig(&SLAM);


    ig.do_rectify = false;
    
    
    if(ig.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(config_dir, cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,ig.M1l,ig.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,ig.M1r,ig.M2r);
    }

    
    cout << "------------------------------------" << endl;
    cout << "Inicio del sistema SLAM, iniciando nodos subscriptores" << endl;
    cout << "------------------------------------" << endl << endl;   
    
    ros::NodeHandle nh;

    int buffer_size = 12;

    message_filters::Subscriber<sensor_msgs::Image>  simg_left(nh,"/img/left",10);
    message_filters::Subscriber<sensor_msgs::Image> simg_right(nh,"/img/right",10);
    message_filters::Subscriber<orb_slam2::msg_mask> sgdata_net(nh,"mask",10);   

    // Esperar al primer mensaje de los suscriptores
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/left");
    ros::topic::waitForMessage<sensor_msgs::Image>("/img/right");
    ros::topic::waitForMessage<orb_slam2::msg_mask>("mask");

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, orb_slam2::msg_mask> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(buffer_size), simg_left,simg_right,sgdata_net);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&ig, _1, _2,_3));

    ros::spin();
  
    
    // stop
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("ORB_SLAM2");
    
    ros::shutdown();
    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight,const orb_slam2::msg_mask::ConstPtr& msg)
{
    cv::Mat imLeft, imRight;
    cv_bridge::CvImageConstPtr cv_ptrLeft,cv_ptrRight;

    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
        cv_ptrRight = cv_bridge::toCvShare(msgRight);

    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
    }
    else
    {
        imLeft = cv_ptrLeft->image;
        imRight = cv_ptrRight->image;
    }
    
    //cout<<"imagen recibida"<<endl;
    int rows =imLeft.rows;
    int cols =imLeft.cols;
    
    cv::Mat mask(rows, cols, CV_8UC1);

    std::memcpy(mask.data, msg->data.data(), msg->data.size());    
    

    mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,mask,msgLeft->header.stamp.toSec());
}
