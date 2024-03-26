/*
 * Author       : Dhanuja Jayasinghe
 * Date         : 25-03-2024
 * Description  : Get the heading from the IMU to estimate the initial heading
 *                (orientation) of the robot within the map  
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <math.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

#define END_VAL     20

int count = 0; 
double heading = 0;
bool map_mode;

//save file location for IMU data
std::string imu_file_path = ros::package::getPath("data_base") + "/robopose/imu_heading.txt";
std::fstream imu_file;

//read file location for robot pose
std::string init_pose_file_path = ros::package::getPath("data_base") + "/robopose/initial.txt";
std::fstream init_pose_file;

/* 
    @brief              callback function for 'imu/robot_heading' subscriber topic
 */
void imu_orientation_cb(const std_msgs::Float64::ConstPtr &msg)
{
    // ignore the first few seconds of data
    if (count < END_VAL) {  
        heading = (double)(msg->data);
        ROS_INFO("Robot Heading: %f, count: %i", heading, count);
    }
    count++;
}

/* 
    @brief                          calculate the relative yaw of the robot w.r.t original heading
    @param      init_heading        inital heading angle of the robot (degrees)
    @param      curr_heading        current heading angle of the robot (degrees)
    @retval                         relative yaw of the robot
 */
double calc_robot_heading (double init_heading, double curr_heading) 
{
    double diff_deg = 0;
    double yaw_r;   
    std::cout << "Initial Heading: " << init_heading << std::endl;
    std::cout << "Current Heading: " << curr_heading << std::endl;

    if (curr_heading >= init_heading && curr_heading) {
        diff_deg = curr_heading - init_heading;
    } else if (curr_heading < init_heading) {
        diff_deg = (curr_heading - init_heading) + 360.00;
    }

    if (diff_deg > 360.00) {
        diff_deg = diff_deg - 360.00;
    }
    std::cout << "Difference: " << diff_deg << std::endl;

    // convert degrees to radians
    yaw_r = diff_deg*(M_PI/180);
    return yaw_r;   // return relative yaw of the robot
}

int main( int argc, char **argv) {
    ros::init(argc, argv, "robot_heading");
    ros::NodeHandle nh;

    // get the current mapping state parameter 
    if (!ros::param::get("map_mode", map_mode)) {
        ROS_ERROR("Failed to retrieve 'map_mode' parameter");
        map_mode = false;
    }

    ros::Subscriber topic_sub = nh.subscribe("/imu/robot_heading", 10, imu_orientation_cb);
    
    while (ros::ok()) {
        if (count > END_VAL) {
            if (map_mode) {
                /*
                    MAPPING PHASE
                        - save the initial heading angle to 'imu_heading.txt'
                */
                imu_file.open(imu_file_path, std::ios::out);
                if (!imu_file.is_open()) {
                    std::cerr << "Error: Could not open \"" << imu_file_path << "\" for writing." << std::endl;
                }
                // write the final heading information to file
                imu_file << std::to_string(heading) << std::endl;

                std::cout << "\u001b[33mInitial Heading: " << heading << "\u001b[37m" <<std::endl;

            } else {
                /*
                    NAVIGATION PHASE
                        - read the initial orientation from 'robotpose/imu_heading.txt' file
                        - calculate the current heading w.r.t. original heading -> initial yaw
                        - write the yaw to 'robot_pose/initial.txt' file
                */
                imu_file.open(imu_file_path, std::ios::in);
                if (!imu_file.is_open()) {
                    std::cerr << "Error: Could not open \"" << imu_file_path << "\" for writing." << std::endl;
                }

                // read the initial heading from the file
                std::string initial_heading;
                std::getline(imu_file, initial_heading);

                // calculate robot heading w.r.t initial heading
                double yaw = calc_robot_heading(std::stod(initial_heading), heading);

                std::cout << "\u001b[33mRobot Yaw: " << yaw << "\u001b[37m" <<std::endl;

                // write the yaw to 'robot_pose/initial.txt'
                init_pose_file.open(init_pose_file_path, std::ios::out);
                if (!init_pose_file.is_open()) {
                    std::cerr << "Error: Could not open \"" << init_pose_file_path << "\" for writing." << std::endl;
                }    
                init_pose_file << "0 0 0 0 0 " << yaw << std::endl;            
            }

            imu_file.close();
            init_pose_file.close();
            // exit the program
            break;
        }
        ros::spinOnce();
    }

    return 0;
}