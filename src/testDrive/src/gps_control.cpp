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
#include <coord.hpp>
#include <testDrive/carInfo.h>
#include <csv.hpp>

using namespace std;

float look_distance = 7.0; // lookahead 거리
float wheel_base = 2.5;    // 차량의 휠베이스
bool flag = true;
int closest_point_index = 0;
double steering_angle = 0;

float current_x, current_y, heading;
ros::Publisher ctrl_cmd_pub;
ros::Subscriber carInfoSubscriber;
vector<vector<float>> ref_path;
string mode;

void setStartingPoint()
{
    float tmpDist = 10;
    float tmpIndex = 0;
    for (int index = 0; index < ref_path.size(); index++)
    {
        float distance = getDistance(ref_path[index][0], current_x, ref_path[index][1], current_y);
        if (distance < tmpDist)
        {
            tmpDist = distance;
            tmpIndex = index;
        }
    }
    closest_point_index = tmpIndex;
    flag = false;
}

vector<float> findTargetPoint(float current_x, float current_y)
{
    float min_distance = numeric_limits<float>::max();
    for (int i = closest_point_index; i < closest_point_index + 300 && i < ref_path.size(); ++i)
    {
        float distance = getDistance(ref_path[i][0], current_x, ref_path[i][1], current_y);
        if (distance < min_distance)
        {
            min_distance = distance;
            closest_point_index = i;
        }
    }
    for (int i = closest_point_index; i < ref_path.size(); ++i)
    {
        float distance = getDistance(ref_path[i][0], current_x, ref_path[i][1], current_y);
        if (distance >= look_distance)
        {
            return ref_path[i];
        }
    }
    return ref_path.back();
}

double clcSteerAngle(float current_x, float current_y, const vector<float> &target_point)
{
    float dx = target_point[0] - current_x;
    float dy = target_point[1] - current_y;
    float distance = getDistance(dx, dy);

    if (distance < 1e-6)
        return 0.0;

    float heading_rad = degToRad(heading);
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

void getInfoData(const testDrive::carInfo &data)
{
    mode = data.mode;
    UTM utm;
    ENU enu;
    current_x = data.utm_x;
    current_y = data.utm_y;

    mode == "ENU" ? heading = 90 - radToDeg(data.yaw) : heading = data.heading;
    cout << "Heading : " << heading << endl;
    // cout << "========================" << endl;
    // cout << "Mode : " << data.mode << endl;
    // cout << "UTM_X : " << data.utm_x << endl;
    // cout << "UTM_Y : " << data.utm_y << endl;
    // cout << "HEADING : " << data.heading << endl;
    // cout << "YAW : " << data.yaw << endl;

    if (flag)
    {
        setStartingPoint();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);
    ref_path = getPath("/src/testDrive/path/K-City.csv");
    carInfoSubscriber = node.subscribe("/info", 1, getInfoData);
    ctrl_cmd_pub = node.advertise<morai_msgs::CtrlCmd>("/ctrl_cmd", 1);
    while (ros::ok())
    {
        pubCtrlCmd();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
