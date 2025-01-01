#include <iostream>
#include <csv.hpp>
#include <cmath>
#include <limits>

using namespace std;
vector<vector<float>> path;

struct Segment
{
    float a, b, c, d;
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

vector<Frenet> frenetCoord()
{
    vector<Frenet> frenet_point;
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

        Point ref_point = findClosestPoint(current_point);

        Point point_T = tangent(current_point, next_point);
        Point point_N = normal(point_T);
        s += normalize(current_point, next_point);
        float d = ((current_point.x - ref_point.x) * point_N.x +
                   (current_point.y - ref_point.y) * point_N.y) /
                  normalize(point_N.x, point_N.y);
        if (d != 0)
        {
            Frenet point;
            point.s = s;
            point.d = d;

            frenet_point.push_back(point);
        }
    }
    return frenet_point;
};

vector<Segment> cubicSpline(vector<Point> pathPoint)
{
    int n = pathPoint.size() - 1;
    vector<float> h(n), alpha(n), l(n + 1), mu(n + 1), z(n + 1);
    vector<Segment> segments;

    for (int i = 0; i < n; i++)
    {
        h[i] = pathPoint[i + 1].x - pathPoint[i].x;
    }

    for (int j = 1; j < n; j++)
    {
        alpha[j] = (3.0 / h[j]) * (pathPoint[j + 1].y - pathPoint[j].y) - (3.0 / h[j - 1]) * (pathPoint[j].y - pathPoint[j - 1].y);
    }

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < n; ++i)
    {
        l[i] = 2.0 * (pathPoint[i + 1].x - pathPoint[i - 1].x) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }
    l[n] = 1.0;
    z[n] = 0.0;

    vector<float> a(n, 0.0), b(n, 0.0), c(n + 1, 0.0), d(n, 0.0);
    c[n] = 0.0;
    for (int k = n - 1; k >= 0; k--)
    {
        c[k] = z[k] - mu[k] * c[k + 1];
        b[k] = (pathPoint[k + 1].y - pathPoint[k].y) / h[k] - h[k] * (c[k + 1] + 2.0 * c[k]) / 3.0;
        d[k] = (c[k + 1] - c[k]) / (3.0 * h[k]);
        a[k] = pathPoint[k].y;

        Segment segment;
        segment.a = a[k];
        segment.b = b[k];
        segment.c = c[k];
        segment.d = d[k];

        segments.push_back(segment);
    }

    return segments;
}

float calcY(vector<Point> pathPoints, vector<Segment> segments, float targetX)
{
    int n = pathPoints.size() - 1;
    int i = -1;
    for (int j = 0; j < n; ++j)
    {
        if (targetX >= pathPoints[j].x && targetX <= pathPoints[j + 1].x)
        {
            i = j;
            break;
        }
    }

    if (i == -1)
    {
        if (targetX < pathPoints.front().x)
        {
            i = 0;
        }
        else if (targetX > pathPoints.back().x)
        {
            i = n - 1;
        }
        else
        {
            cerr << "Error: targetX not within bounds." << endl;
            return 0.0;
        }
    }

    double dx = targetX - pathPoints[i].x;
    return segments[i].a + segments[i].b * dx + segments[i].c * dx * dx + segments[i].d * dx * dx * dx;
};

vector<Point> removeDuplicatePoints(const vector<Point> &points)
{
    vector<Point> uniquePoints;
    for (const auto &point : points)
    {
        bool isDuplicate = false;
        for (const auto &uniquePoint : uniquePoints)
        {
            if (point.x == uniquePoint.x || point.y == uniquePoint.y)
            {
                isDuplicate = true;
                break;
            }
        }
        if (!isDuplicate)
        {
            uniquePoints.push_back(point);
        }
    }
    return uniquePoints;
}

vector<Frenet> changeToFrenet(vector<Point> pathPoint, vector<Segment> segments)
{
    vector<Frenet> frenetPath;
    float s = 0.0;

    for (int i = 0; i < pathPoint.size(); i++)
    {
        float x = pathPoint[i].x;
        float y = pathPoint[i].y;

        float splineY = calcY(pathPoint, segments, x);
        if (i > 0)
        {
            float dx = pathPoint[i].x - pathPoint[i - 1].x;
            float dy = pathPoint[i].y - pathPoint[i - 1].y;
            s += sqrt(dx * dx + dy * dy);
        }

        double d = y - splineY;

        Frenet frenet;
        frenet.s = s;
        frenet.d = d;

        frenetPath.push_back(frenet);
    }

    return frenetPath;
}

vector<int> getHighVarianceIndices(const vector<Frenet> &frenetPath, double threshold)
{
    vector<int> highVarianceIndices;

    for (size_t i = 1; i < frenetPath.size(); ++i)
    {
        double deltaD = abs(frenetPath[i].d - frenetPath[i - 1].d);
        if (deltaD > threshold)
        {
            highVarianceIndices.push_back(i); // 변화량이 큰 인덱스를 저장
        }
    }

    return highVarianceIndices;
}

Point frenetToXY(double s, double d, const vector<Segment> &segments, const vector<Point> &path)
{
    double accumulatedS = 0.0;
    int segmentIndex = -1;

    // Step 1: s에 해당하는 기준 경로 점 찾기
    for (int i = 1; i < path.size(); ++i)
    {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        double segmentLength = sqrt(dx * dx + dy * dy);

        if (accumulatedS + segmentLength >= s)
        {
            segmentIndex = i - 1;
            break;
        }
        accumulatedS += segmentLength;
    }

    if (segmentIndex == -1)
    {
        cerr << "Error: s is out of range of the path." << endl;
        return {0.0, 0.0};
    }

    // Step 2: 기준 경로에서 정확한 (x, y) 계산
    double remainingS = s - accumulatedS;
    double dx = path[segmentIndex + 1].x - path[segmentIndex].x;
    double dy = path[segmentIndex + 1].y - path[segmentIndex].y;
    double segmentLength = sqrt(dx * dx + dy * dy);

    // 기준 점
    double baseX = path[segmentIndex].x + (remainingS / segmentLength) * dx;
    double baseY = path[segmentIndex].y + (remainingS / segmentLength) * dy;

    // Step 3: 법선 벡터 계산
    double tangentX = dx / segmentLength;
    double tangentY = dy / segmentLength;
    double normalX = -tangentY;
    double normalY = tangentX;

    // Step 4: 법선 방향으로 d 적용
    Point xy;
    xy.x = baseX + d * normalX;
    xy.y = baseY + d * normalY;

    return xy;
}

vector<Point> convertFrenetToXY(const vector<Frenet> &frenetPath, const vector<Segment> &segments, const vector<Point> &path)
{
    vector<Point> xyPath;

    for (const auto &frenet : frenetPath)
    {
        Point xy = frenetToXY(frenet.s, frenet.d, segments, path);
        xyPath.push_back(xy);
    }

    return xyPath;
}

int main()
{
    path = getPath("/src/testDrive/path/K-City.csv");
    vector<Point> pathPoint;
    for (int index = 0; index < path.size(); index++)
    {
        Point point;
        point.x = path[index][0];
        point.y = path[index][1];
        pathPoint.push_back(point);
    }
    pathPoint = removeDuplicatePoints(pathPoint);
    vector<Segment> cubicSegments = cubicSpline(pathPoint);

    cout << "Path Point : " << pathPoint.size() << endl;
    cout << "Cubic Segments : " << cubicSegments.size() << endl;
    vector<Frenet> tnbPath = changeToFrenet(pathPoint, cubicSegments);

    // // d 값 변화량이 큰 구간 탐지
    // double threshold = 2000; // 예: 변화량 임계값 설정
    // vector<int> highVarianceIndices = getHighVarianceIndices(tnbPath, threshold);

    // // 탐지된 인덱스 출력
    // cout << "Indices with high d variance:" << endl;
    // for (int idx : highVarianceIndices)
    // {
    //     cout << "Index: " << idx << ", s: " << tnbPath[idx].s << ", d: " << tnbPath[idx].d << endl;
    // }
    // saveFrenetToCSV(tnbPath, "/src/testDrive/path/K-City-frenet.csv");

    vector<Point> xyPath = convertFrenetToXY(tnbPath, cubicSegments, pathPoint);
    saveXYToCSV(xyPath, "/src/testDrive/path/K-City-reconstructed.csv");

    return 0;
}