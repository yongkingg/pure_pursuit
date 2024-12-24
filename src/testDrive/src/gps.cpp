#include <iostream>
#include <vector>
#include <string>
#include <GeographicLib/UTMUPS.hpp>
#include <iomanip>
#include <coord.hpp>
#include <fstream>
#include <ros/ros.h>
#include <morai_msgs/GPSMessage.h>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <pthread.h>

using namespace std;

// =========================== Receiver =========================== //
void getImuData(const sensor_msgs::Imu::ConstPtr &imuData)
{
    Quaternion q;
    q.w = imuData->orientation.w;
    q.x = imuData->orientation.x;
    q.y = imuData->orientation.y;
    q.z = imuData->orientation.z;

    Euler enu_euler = quatToEnuEuler(q);
    cout << "===================================" << endl;
    cout << "yaw : " << enu_euler.yaw * 180 / M_PI << " 도" << endl;
}

void getGPSData(const morai_msgs::GPSMessage::ConstPtr &gpsData)
{
    WGS84 wgs84;
    wgs84.latitude = gpsData->latitude;
    wgs84.longitude = gpsData->longitude;
    wgs84.altitude = 0;
    ENU enu = wgs84ToENU(wgs84);
    cout << "East : " << enu.East << endl;
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    double current_x = msg->position.x;
    double current_y = msg->position.y;
    double heading = msg->heading;
    cout << "Current X : " << current_x << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps");
    ros::NodeHandle node;
    ros::Rate rate(50);
    ros::Subscriber imuSubscriber = node.subscribe("/imu", 1, getImuData);
    ros::Subscriber gpsSubscriber = node.subscribe("/gps", 1, getGPSData);
    ros::Subscriber egoSubscriber = node.subscribe("/Ego_topic", 1, getEgoData);

    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
