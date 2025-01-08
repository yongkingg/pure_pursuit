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
#include <chrono>
#include <testDrive/carInfo.h>

using namespace std;
using namespace std::chrono;

// 1. ego로 넘어오는 위치 정보 : utm
// 2. 내가 전환한 위치정보 : enu
// 3.
// =========================== Receiver =========================== //
Euler enu_euler;
ENU enu;
UTM utm;
double ego_heading = 0.0;
time_point<steady_clock> imu_last_time;
time_point<steady_clock> gps_last_time;
time_point<steady_clock> ego_last_time;

const double time_limit = 1.0;

// IMU 데이터 수신 콜백
void getImuData(const sensor_msgs::Imu::ConstPtr &imuData)
{
    Quaternion q;
    q.w = imuData->orientation.w;
    q.x = imuData->orientation.x;
    q.y = imuData->orientation.y;
    q.z = imuData->orientation.z;

    enu_euler = quatToEnuEuler(q);
    imu_last_time = steady_clock::now();
}

void getGPSData(const morai_msgs::GPSMessage::ConstPtr &gpsData)
{
    WGS84 wgs84;
    wgs84.latitude = gpsData->latitude;
    wgs84.longitude = gpsData->longitude;
    wgs84.altitude = 0;
    enu = wgs84ToENU(wgs84);
    utm = enuToUTM(enu);
    gps_last_time = steady_clock::now();
}

// Ego 데이터 수신 콜백
void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    utm.utm_x = msg->position.x;
    utm.utm_y = msg->position.y;
    ego_heading = msg->heading;
    ego_last_time = steady_clock::now();
}

// 수신 상태 확인 함수
bool isRunning(const time_point<steady_clock> &last_time)
{
    auto now = steady_clock::now();
    double elapsed = duration_cast<seconds>(now - last_time).count();
    return elapsed <= time_limit;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps");
    ros::NodeHandle node;
    ros::Rate rate(50);

    ros::Subscriber imuSubscriber = node.subscribe("/imu", 1, getImuData);
    ros::Subscriber gpsSubscriber = node.subscribe("/gps", 1, getGPSData);
    ros::Subscriber egoSubscriber = node.subscribe("/Ego_topic", 1, getEgoData);

    imu_last_time = steady_clock::now();
    gps_last_time = steady_clock::now();
    ego_last_time = steady_clock::now();

    ros::Publisher infoPublisher = node.advertise<testDrive::carInfo>("/info", 1);
    testDrive::carInfo carInfo;
    while (ros::ok())
    {
        ros::spinOnce();

        bool imu_status = isRunning(imu_last_time);
        bool gps_status = isRunning(gps_last_time);
        bool ego_status = isRunning(ego_last_time);

        if (imu_status && gps_status)
        {
            carInfo.mode = "ENU";
            carInfo.yaw = enu_euler.yaw;
            carInfo.heading = 0.0;
        }
        else if (ego_status)
        {
            carInfo.mode = "UTM";
            carInfo.heading = ego_heading;
            carInfo.yaw = 0.0;
        }
        carInfo.utm_x = utm.utm_x;
        carInfo.utm_y = utm.utm_y;

        cout << "========================" << endl;
        cout << "Mode : " << carInfo.mode << endl;
        cout << "1 UTM_X : " << carInfo.utm_x << endl;
        cout << "2 UTM_Y : " << carInfo.utm_y << endl;
        cout << "HEADING : " << carInfo.heading << endl;
        cout << "YAW : " << carInfo.yaw << endl;

        infoPublisher.publish(carInfo);
        rate.sleep();
    }

    return 0;
}
