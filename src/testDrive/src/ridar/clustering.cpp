#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

void getRidarData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle node;
    ros::Subscriber ridarSubscriber = node.subscribe<sensor_msgs::PointCloud2>("/lidar3D", 1, getRidarData);
    ros::spin();
    return 0;
}