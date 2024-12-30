#ifndef MY_RIDAR
#define MY_RIDAR

#include <iostream>
#include <vector>
#include <sensor_msgs/PointCloud2.h>

using namespace std;

struct Point
{
    float x;
    float y;
    float z;

    Point(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}
};

Point extractPoint(const sensor_msgs::PointCloud2 &cloud_msg, int index, int x_offset, int y_offset, int z_offset, int point_step)
{
    const uint8_t *data_ptr = cloud_msg.data.data();

    // data가 uint8_t 타입의 주소로 저장되어있어 역변환 해줘야 좌표를 읽을 수 있다.
    const float *x = reinterpret_cast<const float *>(data_ptr + index * point_step + x_offset);
    const float *y = reinterpret_cast<const float *>(data_ptr + index * point_step + y_offset);
    const float *z = reinterpret_cast<const float *>(data_ptr + index * point_step + z_offset);

    return Point(*x, *y, *z);
}

#endif