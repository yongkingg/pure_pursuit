
#include <ros/ros.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <fstream>

using namespace std;
std::ofstream csv_file;

// 경로딸때 두개의 dist를 비교해서 너무 작은 값은 굳이 안넣어도 된다..... 이 로직을 추가하자.
void egoTopicCallback(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    ROS_INFO("Received data: x = %f, y = %f", msg->position.x, msg->position.y); // 데이터 확인용
    if (csv_file.is_open())
    {
        // CSV 파일에 데이터 기록
        csv_file
            << msg->position.x << ","
            << msg->position.y << std::endl;
    }
    else
    {
        ROS_ERROR("CSV 파일이 열려있지 않습니다.");
    }
}

int main(int argc, char **argv)
{
    // ROS 노드 초기화
    ros::init(argc, argv, "ego_topic_to_csv");
    ros::NodeHandle nh;

    // CSV 파일 열기
    std::string output_file = "/home/autonav/Desktop/pure_pursuit/src/testDrive/src/output2.csv";
    csv_file.open(output_file, std::ios::out);

    if (!csv_file.is_open())
    {
        ROS_ERROR("CSV 파일을 열 수 없습니다: %s", output_file.c_str());
        return -1;
    }

    ros::Subscriber sub = nh.subscribe("/Ego_topic", 1000, egoTopicCallback);

    // ROS 루프 실행
    ros::spin();

    // CSV 파일 닫기
    csv_file.close();

    return 0;
}
