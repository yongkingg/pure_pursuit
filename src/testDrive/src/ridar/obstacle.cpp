#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/GPSMessage.h>
#include <coord.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg, toROSMsg
#include <pcl/point_types.h>                 // PointXYZ
#include <morai_msgs/EgoVehicleStatus.h>
#include <iomanip>
using namespace std;
UTM utm;
Euler enu_euler;
WGS84 wgs84;
double east_offset, north_offset;

struct CloudPoint
{
    float x, y, z;
};

void getRidarData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    vector<CloudPoint> points;
    for (const auto &point : cloud.points)
    {
        points.push_back({point.x, point.y, point.z});
    }

    // std::cout << "ENU Euler Angles (radians):"
    //           << " roll = " << enu_euler.roll
    //           << ", pitch = " << enu_euler.pitch
    //           << ", yaw = " << enu_euler.yaw << std::endl;

    // cout << "roll : " << enu_euler.roll << " pitch : " << enu_euler.pitch << " yaw : " << enu_euler.yaw << endl;
    Eigen::Matrix3d R, Rx, Ry, Rz;

    Rx << 1, 0, 0,
        0, cos((enu_euler.roll)), -sin((enu_euler.roll)),
        0, sin((enu_euler.roll)), cos((enu_euler.roll));
    Ry << cos((enu_euler.pitch)), 0, sin((enu_euler.pitch)),
        0, 1, 0,
        -sin((enu_euler.pitch)), 0, cos((enu_euler.pitch));
    Rz << cos((enu_euler.yaw)), -sin((enu_euler.yaw)), 0,
        sin((enu_euler.yaw)), cos((enu_euler.yaw)), 0,
        0, 0, 1;
    R = Rz * Ry * Rx;

    vector<ENU> enu_points;
    vector<UTM> utm_points;
    for (int index = 0; index < points.size(); index++)
    {
        Eigen::Vector3d body_point(points[index].x, points[index].y, points[index].z);
        Eigen::Vector3d enu_point = R * body_point;
        enu_points.push_back({enu_point.x(), enu_point.y(), enu_point.z()});
        utm_points.push_back(enuToUTM({enu_point.x(), enu_point.y(), enu_point.z()}));
    }

    for (int i = 0; i < points.size(); i++)
    {
        cout << "Obstacle X : " << utm_points[i].utm_x << " Y : " << utm_points[i].utm_y << endl;
    }
}

void getGPSData(const morai_msgs::GPSMessage::ConstPtr &msg)
{
    WGS84 wgs84;
    wgs84.latitude = msg->latitude;
    wgs84.longitude = msg->longitude;
    wgs84.altitude = msg->altitude;

    east_offset = msg->eastOffset;
    north_offset = msg->northOffset;
    ENU enu = wgs84ToENU(wgs84);
}

void getImuData(const sensor_msgs::Imu::ConstPtr &imuData)
{
    Quaternion q;
    q.w = imuData->orientation.w;
    q.x = imuData->orientation.x;
    q.y = imuData->orientation.y;
    q.z = imuData->orientation.z;

    enu_euler = quatToEnuEuler(q);
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &data)
{
    ENU enu = utmToEnu(data->position.x, data->position.y);
    // cout << data->position.x + east_offset << " " << data->position.y + north_offset << endl;
}

int main(int argc, char **argv)
{
    cout << fixed << setprecision(15);
    ros::init(argc, argv, "ransac");
    ros::NodeHandle node;
    ros::Subscriber egoSub = node.subscribe("/Ego_topic", 1, getEgoData);
    ros::Subscriber ridarSub = node.subscribe<sensor_msgs::PointCloud2>("/2_1_RANSAC_others_PCL2", 1, getRidarData);
    ros::Subscriber gpsSub = node.subscribe("/gps", 1, getGPSData);
    ros::Subscriber imuSub = node.subscribe("/imu", 1, getImuData);
    ros::spin();
    return 0;
}
