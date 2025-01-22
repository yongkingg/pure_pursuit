#include <iostream>
#include <vector>
#include <cmath>
#include <csv.hpp>
#include <eigen3/Eigen/Dense>
using namespace std;
struct ENU
{
    float east, north, up;
};

vector<ENU> initPath(string filePath)
{
    ifstream path(filePath);
    vector<ENU> ref_path;
    string line = "";
    float east, north, up;
    while (getline(path, line))
    {
        istringstream Point(line);
        Point >> east >> north >> up;
        ref_path.push_back({east, north, up});
    }
    return ref_path;
}

void savePathToCSV(const vector<ENU> &data, const string &outputPath)
{
    ofstream outFile(outputPath);
    if (!outFile.is_open())
    {
        cerr << "Error: Unable to open file for writing: " << outputPath << endl;
        return;
    }

    // 데이터 저장
    for (const auto &point : data)
    {
        outFile << point.east << "," << point.north << "," << point.up << "\n";
    }

    outFile.close();
    cout << "Data saved to " << outputPath << endl;
}

pair<vector<ENU>, vector<ENU>> makeLine(const vector<ENU> path, float offset)
{
    vector<ENU> left, right;
    for (int index = 1; index < path.size() - 1; index++)
    {
        ENU cur = path[index];
        ENU next = path[index + 1];

        float dx = next.east - cur.east;
        float dy = next.north - cur.north;
        float norm = sqrt(dx * dx + dy * dy);

        float unit_dx = dx / norm;
        float unit_dy = dy / norm;

        float perp_dx = -unit_dy; // 수직 방향 x
        float perp_dy = unit_dx;  // 수직 방향 y

        ENU leftPoint = {
            cur.east + perp_dx * offset * 1,
            cur.north + perp_dy * offset * 1,
            cur.up};
        ENU rightPoint = {
            cur.east + perp_dx * offset * -1,
            cur.north + perp_dy * offset * -1,
            cur.up};

        left.push_back(leftPoint);
        right.push_back(rightPoint);
    }
    return {left, right};
}

int getClostIdx(vector<ENU> path, ENU point)
{
    float minDist = numeric_limits<float>::max();
    int tmpIndex = -1;
    for (int index = 0; index < path.size(); index++)
    {
        float dist = sqrt(pow(path[index].east - point.east, 2) + pow(path[index].north - point.north, 2));
        if (dist < minDist)
        {
            minDist = dist;
            tmpIndex = index;
        }
    }
    return tmpIndex;
}

vector<string> classifyPoint(const vector<ENU> &path, const vector<ENU> &points, const vector<ENU> &left, const vector<ENU> &right)
{
    vector<string> result;

    for (int index = 0; index < points.size(); index++)
    {
        bool inLeft = false;
        bool inRight = false;
        int clost_idx = getClostIdx(path, points[index]);
        float px = points[index].east;
        float py = points[index].north;

        // 모든 경로 선분 확인
        for (int i = 0; i < path.size() - 1; i++)
        {
            float path_x1 = path[i].east, path_y1 = path[i].north;
            float path_x2 = path[i + 1].east, path_y2 = path[i + 1].north;

            float left_x1 = left[i].east, left_y1 = left[i].north;
            float left_x2 = left[i + 1].east, left_y2 = left[i + 1].north;

            float right_x1 = right[i].east, right_y1 = right[i].north;
            float right_x2 = right[i + 1].east, right_y2 = right[i + 1].north;

            // 초록색 선(왼쪽)과 기준 경로의 교차 여부 확인
            if ((left_y1 > py) != (left_y2 > py))
            {
                float interX = left_x1 + (left_x2 - left_x1) * (py - left_y1) / (left_y2 - left_y1);
                if (px < interX)
                    inLeft = true;
            }

            // 노란색 선(오른쪽)과 기준 경로의 교차 여부 확인
            if ((right_y1 > py) != (right_y2 > py))
            {
                float interX = right_x1 + (right_x2 - right_x1) * (py - right_y1) / (right_y2 - right_y1);
                if (px < interX)
                    inRight = true;
            }

            // 한 번이라도 true로 판정되면 상태를 유지
            if (inLeft && !inRight)
                break;
        }

        // 분류
        if (inLeft && !inRight)
        {
            result.push_back("Left Zone");
        }
        else if (!inLeft && inRight)
        {
            result.push_back("Right Zone");
        }
        else
        {
            result.push_back("Outside");
        }
    }

    return result;
}

vector<bool> isInsidePath(const vector<ENU> &path, const vector<ENU> &points, const vector<ENU> &left, const vector<ENU> &right)
{
    vector<bool> result;
    for (int index = 0; index < points.size(); index++)
    {
        bool inLeft = false;
        bool inRight = false;
        int clost_idx = getClostIdx(path, points[index]);

        for (int i = max(0, clost_idx - 100); i < min((int)path.size(), clost_idx + 100); i++)
        {
            float path_x = path[i].east;
            float path_y = path[i].north;
            float left_x = left[i].east;
            float left_y = left[i].north;
            float right_x = right[i].east;
            float right_y = right[i].north;

            float px = points[index].east;
            float py = points[index].north;

            // 교차 여부를 판단할 때 분모가 0이 되는 경우를 방지
            // 왼쪽 선분과 교차 여부 확인
            if ((left_y > py) != (right_y > py))
            {
                float interX = (right_x - left_x) * (py - left_y) / (right_y - left_y) + left_x;
                if (px < interX)
                {
                    inLeft = true; // 왼쪽 교차
                }
            }

            // 오른쪽 선분과 교차 여부 확인
            if ((right_y > py) != (left_y > py))
            {
                float interX = (left_x - right_x) * (py - right_y) / (left_y - right_y) + right_x;
                if (px < interX)
                {
                    inRight = true; // 오른쪽 교차
                }
            }

            // 선분 사이에 있는지 확인
            if (inLeft && !inRight)
            {
                result.push_back(true);
                break;
            }
        }

        // 반복이 끝난 후에도 조건이 만족되지 않으면 false 추가
        if (!inLeft || inRight)
        {
            result.push_back(false);
        }
    }
    return result;
}

int main()
{
    // data formating
    vector<ENU> data = initPath("/home/autonav/Desktop/yongjun/pure_pursuit/src/testDrive/path/data.csv");
    vector<ENU> slicedData(data.begin(), data.begin() + 10000);
    vector<ENU> points = initPath("/home/autonav/Desktop/yongjun/pure_pursuit/src/testDrive/path/tmpPoint.csv");
    // savePathToCSV(slicedData, "/home/autonav/Desktop/yongjun/pure_pursuit/src/testDrive/path/sliced_data.csv");
    auto [leftPoints, rightPoints] = makeLine(data, 2);
    vector<bool> result = isInsidePath(data, points, leftPoints, rightPoints);
    vector<string> result2 = classifyPoint(data, points, leftPoints, rightPoints);
    for (auto item : result2)
    {
        cout << item << endl;
    }

    // savePathToCSV(leftPoints, "/home/autonav/Desktop/yongjun/pure_pursuit/src/testDrive/path/left.csv");
    // savePathToCSV(rightPoints, "/home/autonav/Desktop/yongjun/pure_pursuit/src/testDrive/path/right.csv");

    return 0;
}
