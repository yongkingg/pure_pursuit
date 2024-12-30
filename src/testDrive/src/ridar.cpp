#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <ros/ros.h>
#include <limits>
#include <ridar.hpp>
#include <coord.hpp>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/GPSMessage.h>

using namespace std;

double roll, pitch, yaw;
Eigen::Vector3d point_enu;
Eigen::Vector3d vehicle_enu_position(0, 0, 0);
std::vector<Eigen::Vector3d> objects_enu;

void pointToENU(float point_x, float point_y, float point_z)
{
    Eigen::Matrix3d R_roll, R_pitch, R_yaw;
    double phi = roll;
    double theta = pitch;
    double psi = yaw;

    R_roll << 1, 0, 0,
        0, cos(phi), -sin(phi),
        0, sin(phi), cos(phi);

    R_pitch << cos(theta), 0, sin(theta),
        0, 1, 0,
        -sin(theta), 0, cos(theta);

    R_yaw << cos(psi), -sin(psi), 0,
        sin(psi), cos(psi), 0,
        0, 0, 1;

    Eigen::Matrix3d R = R_yaw * R_pitch * R_roll;

    Eigen::Vector3d point_body(point_x, point_y, point_z);
    point_enu = R * point_body + vehicle_enu_position;
}

void getRiDARData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const int point_step = msg->point_step;
    const int point_count = msg->data.size() / point_step;

    int x_offset = -1, y_offset = -1, z_offset = -1;
    for (const auto &field : msg->fields)
    {
        if (field.name == "x")
            x_offset = field.offset;
        if (field.name == "y")
            y_offset = field.offset;
        if (field.name == "z")
            z_offset = field.offset;
    }

    // 필드가 존재하지 않을 경우 에러 출력
    if (x_offset == -1 || y_offset == -1 || z_offset == -1)
    {
        ROS_ERROR("x, y, z fields not found in PointCloud2 message!");
        return;
    }

    for (int index = 0; index < point_count; index++)
    {
        Point point = extractPoint(*msg, index, x_offset, y_offset, z_offset, point_step);
        if (!(point.x == 0 && point.y == 0 && point.z == 0))
        {
            pointToENU(point.x, point.y, point.z);
        }
    }
}

void getImuData(const sensor_msgs::Imu::ConstPtr &imuData)
{
    Quaternion q;
    q.w = imuData->orientation.w;
    q.x = imuData->orientation.x;
    q.y = imuData->orientation.y;
    q.z = imuData->orientation.z;
    Euler enu_euler = quatToEnuEuler(q);

    roll = enu_euler.roll;
    pitch = enu_euler.pitch;
    yaw = enu_euler.yaw;
}

void getObjectInfoData(const morai_msgs::ObjectStatusList::ConstPtr &msg)
{
    objects_enu.clear();
    for (const auto &object : msg->npc_list)
    {
        double object_x = object.position.x;
        double object_y = object.position.y;
        double object_z = object.position.z;

        objects_enu.push_back(Eigen::Vector3d(object_x, object_y, object_z));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ridar_coord");
    ros::NodeHandle node;
    ros::Rate rate(10);

    // 차량 ENU 위치 설정
    vehicle_enu_position = Eigen::Vector3d(15.5, 1100, 1);

    ros::Subscriber lidar_sub = node.subscribe("/0_UDP_rawdata", 1, getRiDARData);
    ros::Subscriber imu_sub = node.subscribe("/imu", 1, getImuData);
    ros::Subscriber object_info_sub = node.subscribe("/Object_topic", 1, getObjectInfoData);
    while (ros::ok())
    {
        for (const auto &object : objects_enu)
        {
            cout << "Object Position (ENU): x = " << object.x()
                 << ", y = " << object.y()
                 << ", z = " << object.z() << endl;
        }

        cout << "PointCloud ENU Position : x = " << point_enu.x()
             << ", y = " << point_enu.y()
             << ", z = " << point_enu.z() << endl;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
