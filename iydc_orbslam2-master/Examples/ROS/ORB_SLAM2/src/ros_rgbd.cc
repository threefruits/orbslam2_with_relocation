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
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <glog/logging.h>
using namespace std;

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

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {     
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings is_gui" << endl;
    // if(argc != 6)
    // {     
    //     cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings do_rectify is_gui path_to_map" << endl;
        ros::shutdown();
        return 1;
    }    

    bool bisgui;
    stringstream isgui(argv[3]);
	isgui >> boolalpha >> bisgui;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,bisgui);
    //ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,bisgui,ORB_SLAM2::System::LocalizationOnly,argv[5]);
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
   
    // by sj
    vision_path_pub = nh.advertise<nav_msgs::Path>("/vision_path", 1);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vision_pose", 1);
    true_path_pub = nh.advertise<nav_msgs::Path>("/true_path", 1);
    true_pose_sub = nh.subscribe("/vrpn_client_node/three/pose", 100, true_poseCallback);
    //

    ros::spin();
    LOG(INFO) << "Saving Map...";
    // Stop all threads
    SLAM.Shutdown();
    SLAM.SaveMap("MapPointandKeyFrame.map");
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
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
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Tcw=mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    
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


