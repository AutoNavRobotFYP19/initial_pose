/*
 * Author       : Dhanuja Jayasinghe
 * Date         : 25-03-2024
 * Description  : Get the heading of the robot from the IMU to estimate
 *                the initial heading (orientation) of the robot within the map  
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <vector>

#define START_VAL   10
#define END_VAL     40

int count = 0; 
double heading = 0;

//save file location for IMU data
std::string file_path = ros::package::getPath("data_base") + "/robopose/imu_heading.txt";
std::ofstream file;

//callback function for IMU orientation data
void imu_orientation_cb(const std_msgs::Float64::ConstPtr &msg)
{
    // ignore the first 2s of data
    if (count >= START_VAL && count < END_VAL) {  
        heading = (double)(msg->data);
        ROS_INFO("Robot Heading: %f, count: %i", heading, count);
    }
    count++;
}

int main( int argc, char **argv) {
    ros::init(argc, argv, "robot_heading");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub = nh.subscribe("/imu/robot_heading", 10, imu_orientation_cb);
    
    while (ros::ok()) {
        if (count > END_VAL) {
            //save the final heading information to file
            file.open(file_path, std::ios::out);
            if (!file.is_open()) {
                std::cerr << "Error: Could not open \"" << file_path << "\" for writing." << std::endl;
            }
            file << std::to_string(heading) << std::endl;
            file.close();
            // exit the program
            break;
        }
        ros::spinOnce();
    }


    return 0;
}