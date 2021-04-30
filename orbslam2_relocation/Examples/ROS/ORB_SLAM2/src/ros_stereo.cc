/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <glog/logging.h>

// publish Tcw to rviz view
cv::Mat Tcw;
ros::Publisher vision_path_pub;
ros::Publisher vision_pose_pub;
ros::Publisher true_path_pub;
ros::Subscriber true_pose_sub;

nav_msgs::Path vision_path;
geometry_msgs::PoseStamped vision_pose;
nav_msgs::Path true_path;
geometry_msgs::PoseStamped true_pose;
Eigen::Matrix4d init_pose;
bool hasInitial = false;
void true_poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    true_pose = *msg;
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    double w = true_pose.pose.orientation.w;
    double x = true_pose.pose.orientation.x;
    double y = true_pose.pose.orientation.y;
    double z = true_pose.pose.orientation.z;
    pose.block(0, 0, 3, 3) = Eigen::Quaterniond(w, x, y, z).toRotationMatrix();
    pose(0,3) = true_pose.pose.position.x;
    pose(1,3) = true_pose.pose.position.y;
    pose(2,3) = true_pose.pose.position.z;
    if (!hasInitial) {
        init_pose = pose;
        hasInitial = true;
    }
    pose =  init_pose.inverse() * pose;
    Eigen::Matrix3d mat = pose.block(0,0,3,3);
    Eigen::Quaterniond q(mat);
    true_pose.pose.orientation.w = q.w();
    true_pose.pose.orientation.x = q.x();
    true_pose.pose.orientation.y = q.y();
    true_pose.pose.orientation.z = q.z();
    true_pose.pose.position.x = pose(0,3);
    true_pose.pose.position.y = pose(1,3);
    true_pose.pose.position.z = pose(2,3);
}

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "Stereo");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
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

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    // by sj
    vision_path_pub = nh.advertise<nav_msgs::Path>("/vision_path", 1);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vision_pose", 1);
    true_path_pub = nh.advertise<nav_msgs::Path>("/true_path", 1);
    true_pose_sub = nh.subscribe("/vrpn_client_node/three/pose", 100, true_poseCallback);
    //

    ros::spin();

    // Stop all threads
    //LOG(INFO) << "Stop SLAM...";
    SLAM.Shutdown();
    //LOG(INFO) << "Saving Map...";
    SLAM.SaveMap("MapPointandKeyFrame.map");
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");
    //LOG(INFO) << "ROS shutdown...";
    cerr << "Shutdown orb_ros." << endl;
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{

    // Copy the ros image message to cv::Mat.
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

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    if (!Tcw.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        twc = -Rwc*Tcw.rowRange(0,3).col(3);

        Eigen::Matrix3f  M;
        M << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
            Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
            Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
        Eigen::Quaternionf q(M);

        vision_pose.header.frame_id = "/world";
        //geometry_msgs::PoseStamped vision_pose;
        vision_pose.pose.position.x = twc.at<float>(0,0);
        vision_pose.pose.position.y = twc.at<float>(0,1);
        vision_pose.pose.position.z = twc.at<float>(0,2);
        vision_pose.pose.orientation.w =q.w();
        vision_pose.pose.orientation.x =q.x();
        vision_pose.pose.orientation.y =q.y();
        vision_pose.pose.orientation.z =q.z();
        vision_path.poses.push_back(vision_pose);
    }
    
    vision_pose_pub.publish(vision_pose);

    vision_path.header.frame_id = "/world";
    vision_path_pub.publish(vision_path);
    //
    true_path.poses.push_back(true_pose);
    true_path.header.frame_id = "/world";
    true_path_pub.publish(true_path);
}


