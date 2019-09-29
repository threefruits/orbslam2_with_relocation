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

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include <glog/logging.h>
using namespace std;
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
// publish Tcw to rviz view
cv::Mat Tcw;
ros::Publisher vision_path_pub;
ros::Publisher true_path_pub;
ros::Subscriber true_pose_sub;

nav_msgs::Path vision_path;
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

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

int main(int argc, char **argv)
{
    // Initialize Google's logging library.
    google::InitGoogleLogging(argv[0]);
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    // by sj
    ros::NodeHandle nh;
    vision_path_pub = nh.advertise<nav_msgs::Path>("/vision_path", 1);
    true_path_pub = nh.advertise<nav_msgs::Path>("/true_path", 1);
    true_pose_sub = nh.subscribe("/vrpn_client_node/three/pose", 100, true_poseCallback);
    //
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Tcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    if (Tcw.empty())
        Tcw = cv::Mat::eye(4, 4, CV_32FC1);
    vision_path.header.frame_id = "/world";
    geometry_msgs::PoseStamped vision_pose;
    vision_pose.pose.position.x = Tcw.at<float>(0,3);
    vision_pose.pose.position.y = Tcw.at<float>(1,3);
    vision_pose.pose.position.z = Tcw.at<float>(2,3);
    vision_path.poses.push_back(vision_pose);
    vision_path_pub.publish(vision_path);

    //
    true_path.poses.push_back(true_pose);
    true_path.header.frame_id = "/world";
    true_path_pub.publish(true_path);
}


