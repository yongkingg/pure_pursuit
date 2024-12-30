#include <iostream>
#include <config.hpp>
#include <vector>
#include <cmath>
#include <fstream>
#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <limits>

using namespace std;
vector<vector<float>> path;
struct Point
{
    float x, y;
};

struct frenet
{
    float s, d;
};

float normalize(float x, float y, float z = 0)
{
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
};

float normalize(Point current_point, Point next_point)
{
    return sqrt(pow(current_point.x - next_point.x, 2) + pow(next_point.y - current_point.y, 2));
}

Point tangent(Point current_point, Point next_point)
{
    Point tan_point;
    float tan_x = current_point.x - next_point.x;
    float tan_y = current_point.y - next_point.y;
    float norm = normalize(tan_x, tan_y);
    if (norm == 0)
    {
        tan_point.x = 0;
        tan_point.y = 0;
    }
    else
    {
        tan_point.x = tan_x / norm;
        tan_point.y = tan_y / norm;
    }
    return tan_point;
};

Point normal(Point tan_point)
{
    Point norm_point;
    norm_point.x = -tan_point.y;
    norm_point.y = tan_point.x;
    return norm_point;
};

Point findClosestPoint(Point current_point)
{
    Point closest_point;
    float min_distance = std::numeric_limits<float>::max();

    for (const auto &point : path)
    {
        Point ref_point = {point[0], point[1]};
        float distance = normalize(current_point, ref_point);

        // 자기 자신을 제외하기 위해 거리 0은 건너뜀
        if (distance > 1e-6 && distance < min_distance)
        {
            min_distance = distance;
            closest_point = ref_point;
        }
    }

    return closest_point;
}

vector<frenet> frenetCoord()
{
    vector<frenet> frenet_point;
    float s = 0.0;
    for (int index = 0; (2 * index + 1) < path.size(); index++)
    {
        Point current_point, next_point;
        current_point.x = path[2 * index][0];
        current_point.y = path[2 * index][1];
        next_point.x = path[2 * index + 1][0];
        next_point.y = path[2 * index + 1][1];
        float dx = next_point.x - current_point.x;
        float dy = next_point.y - current_point.y;
        if (dx == 0 || dy == 0)
            continue;
        cout << "==================" << endl;
        cout << "dx: " << dx << ", dy: " << dy << endl;

        Point ref_point = findClosestPoint(current_point);

        Point point_T = tangent(current_point, next_point);
        Point point_N = normal(point_T);
        cout << "Tangent Norm: " << normalize(point_T.x, point_T.y) << endl;
        cout << "Point N.x : " << point_N.x << " Point N.y : " << point_N.y << endl;
        cout << "Norm : " << normalize(point_N.x, point_N.y) << endl;
        cout << "Closest Dist : " << normalize(current_point.x - ref_point.x, current_point.y - ref_point.y) << endl;
        s += normalize(current_point, next_point);
        float d = ((current_point.x - ref_point.x) * point_N.x +
                   (current_point.y - ref_point.y) * point_N.y) /
                  normalize(point_N.x, point_N.y);
        if (d != 0)
        {
            frenet point;
            point.s = s;
            point.d = d;

            frenet_point.push_back(point);
        }
    }
    return frenet_point;
};

void saveFrenetToCSV(const vector<frenet> &frenet_data, const string &file_path)
{
    string full_path;
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
    {
        full_path = string(cwd) + "/" + file_path;
    }
    else
    {
        cerr << "Error: Unable to get current working directory" << endl;
        return;
    }

    cout << "Full path : " << full_path << endl;

    ofstream file_stream(full_path);
    if (!file_stream.is_open())
    {
        cerr << "Error: Could not open file " << full_path << endl;
        return;
    }

    file_stream << fixed << setprecision(15);
    for (const auto &point : frenet_data)
    {
        file_stream << point.s << "," << point.d << "\n";
    }

    file_stream.close();
    cout << "Frenet data saved to " << full_path << endl;
}

int main()
{
    path = getPath("/src/testDrive/path/K-City.csv");
    vector<frenet> tnb_path = frenetCoord();
    // saveFrenetToCSV(tnb_path, "/src/testDrive/path/K-City-frenet.csv");
    return 0;
}