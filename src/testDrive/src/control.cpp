#include <iostream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <pthread.h>
#include <morai_msgs/CtrlCmd.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <ros/package.h>
#include <config.hpp>
#include <myMath.hpp>
#include <csv.hpp>

using namespace std;

float look_distance = 7.0; // lookahead 거리
float wheel_base = 2.5;    // 차량의 휠베이스
bool flag = true;
int closest_point_index = 0;
double steering_angle = 0;

float current_x;
float current_y;
float heading;
ros::Publisher ctrl_cmd_pub;
vector<vector<float>> my_path;

void setStartingPoint()
{
    float tmpDist = 10;
    float tmpIndex = 0;
    for (int index = 0; index < my_path.size(); index++)
    {
        float distance = getDistance(my_path[index][0], current_x, my_path[index][1], current_y);
        if (distance < tmpDist)
        {
            tmpDist = distance;
            tmpIndex = index;
        }
    }
    closest_point_index = tmpIndex;
    flag = false;
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    current_x = msg->position.x;
    current_y = msg->position.y;
    heading = msg->heading;

    if (flag)
    {
        setStartingPoint();
    }
}

void *egoDataThread(void *p_Arg)
{
    ros::NodeHandle *node = (ros::NodeHandle *)p_Arg;
    ros::Subscriber subscribeEgo = node->subscribe("/Ego_topic", 1, getEgoData);
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return nullptr;
}

vector<float> findTargetPoint(float current_x, float current_y)
{
    float min_distance = numeric_limits<float>::max();
    for (int i = closest_point_index; i < closest_point_index + 300 && i < my_path.size(); ++i)
    {
        float distance = getDistance(my_path[i][0], current_x, my_path[i][1], current_y);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_point_index = i;
        }
    }
    for (int i = closest_point_index; i < my_path.size(); ++i)
    {
        float distance = getDistance(my_path[i][0], current_x, my_path[i][1], current_y);
        if (distance >= look_distance)
        {
            return my_path[i];
        }
    }
    return my_path.back();
}

double clcSteerAngle(float current_x, float current_y, const vector<float> &target_point)
{
    float dx = target_point[0] - current_x;
    float dy = target_point[1] - current_y;
    float distance = getDistance(dx, dy);

    if (distance < 1e-6)
        return 0.0;

    float heading_rad = heading * M_PI / 180.0;
    float alpha = atan2(dy, dx) - heading_rad;

    double steering_angle = atan2(2.0 * wheel_base * sin(alpha), look_distance);
    return steering_angle;
}

void pubCtrlCmd()
{
    vector<float> target_point = findTargetPoint(current_x, current_y);
    steering_angle = clcSteerAngle(current_x, current_y, target_point);
    morai_msgs::CtrlCmd ctrl_cmd;
    ctrl_cmd.longlCmdType = 2;
    ctrl_cmd.velocity = 20.0;
    ctrl_cmd.steering = steering_angle;
    ctrl_cmd_pub.publish(ctrl_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    my_path = getPath("/src/testDrive/path/K-City.csv");
    pthread_t getEgoThread;
    pthread_create(&getEgoThread, NULL, egoDataThread, &node);

    ctrl_cmd_pub = node.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    while (ros::ok())
    {
        pubCtrlCmd();
        loop_rate.sleep();
    }

    pthread_join(getEgoThread, NULL);
    return 0;
}
