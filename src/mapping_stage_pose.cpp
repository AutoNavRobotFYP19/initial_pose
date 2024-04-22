/*
 * Author       : Dhanuja Jayasinghe
 * Date         : 04-04-2024
 * Description  : Get the starting pose of the robot (GPS location)
 */

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/NavSatFix.h"      // for GPS data (lat, long, alt)
#include "geometry_msgs/PointStamped.h" // for GPS data (x,y,z)
#include <math.h>

#include <ros/package.h>
#include <iostream>
#include <fstream>

#define START_VAL   5
#define END_VAL     20

//save file location for IMU data
std::string gps_file_path = ros::package::getPath("data_base") + "/robopose/rtk_init.txt";
std::fstream gps_file;

// pose initialization
std::string init_file_path = ros::package::getPath("data_base") + "/robopose/initial.txt";
std::fstream init_file;

int count = 0;
int arr_pos = 0;
double initial_lat[END_VAL - START_VAL];
double initial_long[END_VAL - START_VAL];

double lat;
double lon;
/* 
    @brief              callback function for '/robot_rtk_gps' subscriber topic
 */
void gps_location_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{   
    // ignore the first few seconds of data
    if (count >= START_VAL && count < END_VAL) {
        initial_lat[arr_pos] = (double)msg->latitude;
        initial_long[arr_pos] = (double)msg->longitude;
        // lat = msg->latitude;
        // lon = msg->longitude;
        ROS_INFO("Latitude: %.12f, Longitude: %.12f", msg->latitude, msg->longitude);
        arr_pos++;
    } else {
        ROS_INFO("Waiting for stable GPS data");
    }
    count++;
}

/* 
    @brief              calculate the average value of an array of length n
    @param      arr     array containing values
    @param      n       length of the array
    @retval     avg     calculated average
 */
double calc_average (double *arr, int n) 
{
    double avg = 0;

    for (int i = 0; i < n; i++) {
        avg += arr[i];
    }

    return avg/n;
}


int main( int argc, char **argv) {
    ros::init(argc, argv, "initial_gps_location");
    ros::NodeHandle nh;

    ros::Subscriber topic_sub = nh.subscribe("/robot_rtk_gps", 10, gps_location_cb);

    // set initial pose to '0 0 0 0 0 0'
    init_file.open(init_file_path, std::ios::out);
    if (!init_file.is_open()) {
        std::cerr << "Error: Could not open \"" << init_file_path << "\" for writing." << std::endl;
    } 

    init_file << "0 0 0 0 0 0" << std::endl;
    init_file.close();

    
    while (ros::ok()) {
        if (count >= END_VAL) {
            // calculate the mean value for latitudes and longitudes
            lat = calc_average(initial_lat, END_VAL - START_VAL);
            lon = calc_average(initial_long, END_VAL - START_VAL);

            ROS_INFO("\u001b[36mFinal Latitude: %f, Longitude: %f\u001b[37m", lat, lon);

            // save the initial gps location to 'initial_gps_location.txt'
            gps_file.open(gps_file_path, std::ios::out);
            if (!gps_file.is_open()) {
                std::cerr << "Error: Could not open \"" << gps_file_path << "\" for writing." << std::endl;
            } 

            // std::string s_lat = std::to_string(lat);
            // std::string s_lon = std::to_string(lon);
            // std::string s_alt = "22.529253";
            double alt = 22.529253;

            gps_file <<  std::fixed << std::setprecision(12) << lat << " " << lon << " " << alt << std::endl;



            gps_file.close();
            break;        
        }
        ros::spinOnce();
    }

    return 0;
}