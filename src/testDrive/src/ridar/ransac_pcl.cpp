#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h> // SACSegmentation
#include <pcl/filters/extract_indices.h>       // ExtractIndices
#include <pcl/filters/passthrough.h>

using namespace std;
ros::Publisher obstacles_pub;

void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);                              // inliers : 필터에 매치된 포인트들을 저장하기 위한 변수
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);               // coefficients : 평면 모델의 계수를 저장하는 변수
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>()); // outliers : 필터에 매치되지 않은 포인트들을 저장하기 위한 변수

    // seg : 평면을 탐지하기 위한 세그멘테이션 객체
    // 세그멘테이션 : 포인트 클라우드 데이터를 특정 기하학적 모델에 맞춰 분리하는데에 사용됨
    // pcl::SACSegmentation : Sample Consensus의 약자로, 모델을 포인트 데이터에 적합하게 추정하는데에 사용되는 알고리즘
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);     // 탐지된 모델의 계수를 최적화할지 여부를 설정
    seg.setInputCloud(cloud);              // 탐지할 대상 포인트 클라우드를 설정
    seg.setModelType(pcl::SACMODEL_PLANE); // 탐지 모델의 유형을 설정함. PLANE은 평면이다
    seg.setMethodType(pcl::SAC_RANSAC);    // 모델을 탐지할때 사용할 알고리즘 유형을 설정
    seg.setDistanceThreshold(0.01);        // RANSAC 알고리즘에서 가장 중요한 변수 중 하나인 Threshold를 설정함. (모델에서 0.01m 이내에 있는 점만 inliers로 간주함)
    seg.segment(*inliers, *coefficients);  // segmentation을 수행한 결과를 저장함

    pcl::ExtractIndices<pcl::PointXYZ> extract; // extract : 포인트 클라우드에서 특정 인덱스에 해당하는 점들을 추출하거나 제거하는 작업 수행
    extract.setInputCloud(cloud);               // 탐지할 대상 포인트 클라우드를 설정
    extract.setIndices(inliers);                // inliers의 인덱스들을 세팅함
    extract.setNegative(true);                  // setNegative : 추출 작업의 모드를 Negative로 설정 => true : 인덱스에 속하지 않는 점(Outliers)를 추출함
    extract.filter(*outliers);                  // 필터링 작업을 수행하여 결과를 outliers에 저장함

    pcl::PassThrough<pcl::PointXYZ> pass; // PassThrough : 축과 범위를 기반으로 Point를 추출하는 필터
    pass.setInputCloud(outliers);         // 탐지할 대상 포인트 클라우드 설정
    pass.setFilterFieldName("z");         // 축 설정
    pass.setFilterLimits(-0.8, 1);        // 범위 설정
    pass.setFilterLimitsNegative(false);  // 원하는 범위 내의 point를 통과시킬 것인지. true : 범위 영역 이외를 통과시킴. false : 범위 내부를 통과시킴.
    pass.filter(*outliers);               // 필터링 작업을 수행하여 결과를 outliers에 저장함.

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*outliers, output);
    output.header.frame_id = "map";
    output.header.stamp = ros::Time::now();

    obstacles_pub.publish(output);
    cout << "필터링 전 사이즈 : " << cloud->size() << endl;
    cout << "필터링 결과 사이즈 : " << outliers->size() << endl;
}

void getRidarData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // 포인트 클라우드를 PCL의 PointCloud에 변환
    // 동적할당 받는 이유는, riDAR 특성상 데이터의 크기가 큰데, 이를 메모리 효율적으로 관리하기 위함이다.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    ransac(cloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac");
    ros::NodeHandle node;

    obstacles_pub = node.advertise<sensor_msgs::PointCloud2>("filtered_obstacles", 1);
    ros::Subscriber ridarSub = node.subscribe<sensor_msgs::PointCloud2>("/lidar3D", 1, getRidarData);

    ros::spin();
    return 0;
}
