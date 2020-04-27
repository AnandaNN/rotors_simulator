/*
 * Copyright 2020 Ananda N H Nielsen Technical University of Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h> 
#include <ros/ros.h>
#include <ros/console.h> 
#include <Eigen/Eigen>
#include <stdio.h>
#include <boost/bind.hpp>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>

#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/eigen_mav_msgs.h>


#include <fstream>
#include <iostream>

#define RAD_2_DEG    180 / 3.14159265358979323846  /* pi */

ros::Publisher trajectory_publisher;
ros::Subscriber matlab_subscriber;

void MsgCallback(const geometry_msgs::Twist desiredPose)
{
    // the incoming geometry_msgs::PoseStamped is transformed to a tf::Quaterion
    
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;

    trajectory_msg.header.stamp = ros::Time::now();

    Eigen::Vector3d desired_position(desiredPose.linear.x, desiredPose.linear.y,
                                   desiredPose.linear.z);

    double desired_yaw = desiredPose.angular.z * RAD_2_DEG;

    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);


    // this Vector is then published:
    trajectory_publisher.publish(trajectory_msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion_to_rpy");

    ros::NodeHandle nh;

    trajectory_publisher = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

    matlab_subscriber = nh.subscribe("/matlab/tracejtoryTwist", 1, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_DEBUG("Ready for MATLAB cmd");

    ros::spin();

    return 0;
}
