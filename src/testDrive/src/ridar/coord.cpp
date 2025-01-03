#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <morai_msgs/ObjectStatusList.h>
#include <morai_msgs/GPSMessage.h>
#include <coord.hpp>
#include <pcl/segmentation/sac_segmentation.h> // SACSegmentation
#include <pcl/filters/extract_indices.h>       // ExtractIndices
#include <pcl/filters/passthrough.h>

using namespace std;
ros::Publisher obstacles_pub;
float utm_x, utm_y, utm_z, yaw;

void getRidarData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    Eigen::Matrix4f transformation;
    transformation << 1, 0, 0, 3.0,
        0, 1, 0, 0.0,
        0, 0, 1, 0.8,
        0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*cloud, *trans_cloud, transformation);

    Eigen::Matrix4f word_trans;
    word_trans << cos(yaw), -sin(yaw), 0, utm_x,
        sin(yaw), cos(yaw), 0, utm_y,
        0, 0, 1, utm_z,
        0, 0, 0, 1;

    pcl::PointCloud<pcl::PointXYZ>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*trans_cloud, *world_cloud, word_trans);

    // 변환된 좌표 출력
    std::cout << "Transformed PointCloud:" << std::endl;
    for (const auto &point : world_cloud->points)
    {
        std::cout << "X: " << point.x << ", Y: " << point.y << ", Z: " << point.z << std::endl;
    }
}

void getGPSData(const morai_msgs::GPSMessage::ConstPtr &msg)
{
    WGS84 wgs84;
    wgs84.latitude = msg->latitude;
    wgs84.longitude = msg->longitude;
    wgs84.altitude = msg->altitude;
    ENU enu = wgs84ToENU(wgs84);
    UTM utm = enuToUTM(enu);

    utm_x = utm.utm_x;
    utm_y = utm.utm_y;
    utm_z = 0;
}

void getImuData(const sensor_msgs::Imu::ConstPtr &imuData)
{
    Quaternion q;
    q.w = imuData->orientation.w;
    q.x = imuData->orientation.x;
    q.y = imuData->orientation.y;
    q.z = imuData->orientation.z;

    Euler enu_euler = quatToEnuEuler(q);
    yaw = -enu_euler.yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac");
    ros::NodeHandle node;

    // ROS 발행자 초기화
    obstacles_pub = node.advertise<sensor_msgs::PointCloud2>("filtered_obstacles", 1);

    // ROS 그래픽자 초기화
    ros::Subscriber ridarSub = node.subscribe<sensor_msgs::PointCloud2>("/lidar3D", 1, getRidarData);
    ros::Subscriber gpsSub = node.subscribe("/gps", 1, getGPSData);
    ros::Subscriber imuSub = node.subscribe("/imu", 1, getImuData);

    ros::spin();
    return 0;
}
