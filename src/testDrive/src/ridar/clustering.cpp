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
#include <pcl/segmentation/extract_clusters.h>
#include <morai_msgs/EgoVehicleStatus.h>

using namespace std;
ros::Publisher clustered_pub;
float utm_x, utm_y, utm_z;

void clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // KD-Tree 생성 (클러스터링에서 사용)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    // 클러스터링 실행
    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.5); // 클러스터 거리 임계값 (5cm 설정)
    ec.setMinClusterSize(100);   // 최소 클러스터 크기
    ec.setMaxClusterSize(1500);  // 최대 클러스터 크기
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_clusters(new pcl::PointCloud<pcl::PointXYZRGB>());
    int cluster_id = 0;
    for (const auto &indices : cluster_indices)
    {
        uint8_t r = rand() % 256;
        uint8_t g = rand() % 256;
        uint8_t b = rand() % 256;

        for (const auto &index : indices.indices)
        {
            pcl::PointXYZRGB point;
            point.x = cloud->points[index].x;
            point.y = cloud->points[index].y;
            point.z = cloud->points[index].z;
            point.r = r;
            point.g = g;
            point.b = b;
            merged_clusters->points.push_back(point);
        }

        cluster_id++;
    }

    // 병합된 클러스터를 퍼블리시
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(*merged_clusters, output_msg);
    output_msg.header.frame_id = "map";
    output_msg.header.stamp = ros::Time::now();
    clustered_pub.publish(output_msg);

    cout << "Total clusters: " << merged_clusters->points.size() << endl;
}

void ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*outliers);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(outliers);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-0.8, 1);
    pass.setFilterLimitsNegative(false);
    pass.filter(*outliers);

    clustering(outliers);
}

void getRidarData(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);

    ransac(cloud);
}

void getEgoData(const morai_msgs::EgoVehicleStatus::ConstPtr &msg)
{
    utm_x = msg->position.x;
    utm_y = msg->position.y;
    utm_z = msg->position.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ransac");
    ros::NodeHandle node;

    clustered_pub = node.advertise<sensor_msgs::PointCloud2>("/clustered_object", 1);
    ros::Subscriber positionSub = node.subscribe<morai_msgs::EgoVehicleStatus>("/Ego_topic", 1, getEgoData);
    ros::Subscriber ridarSub = node.subscribe<sensor_msgs::PointCloud2>("/lidar3D", 1, getRidarData);

    ros::spin();
    return 0;
}
