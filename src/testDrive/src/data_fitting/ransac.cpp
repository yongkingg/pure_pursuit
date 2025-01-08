#include <iostream>
#include <vector>
#include <cmath>
#include <csv.hpp>
#include <eigen3/Eigen/Dense>
using namespace std;

// y = ax^2 + bx + c
// ax^2 + bx + c - y = 0
struct Line
{
    float a, b, c;
};

struct Parameters
{
    float threshold;
    int sampleSize, samplingNumber;
};

float getDist(Line line, Point point)
{
    // 곡선의 y값 계산
    float predictedY = line.a * point.x * point.x + line.b * point.x + line.c;
    // 점과 곡선의 y값 차이 계산
    return abs(predictedY - point.y);
}

Line makeLine(vector<Point> points)
{
    // A
    // x1^2 x1 1
    // x2^2 x2 1
    // x3^2 x3 1
    Eigen::Matrix3f A;
    // Y
    // y1
    // y2
    // y3
    Eigen::Vector3f Y;

    for (int index = 0; index < 3; index++)
    {
        float x = points[index].x;
        A(index, 0) = x * x;
        A(index, 1) = x;
        A(index, 2) = 1;
        Y(index) = points[index].y;
    }

    Eigen::Vector3f params = (A.transpose() * A).inverse() * (A.transpose() * Y);
    return {params(0), params(1), params(2)};
}

int getSamplingNumber(int sampleSize)
{
    float alpha = 0.9;
    float P = 0.99;
    int n = sampleSize;
    return log(1 - P) / log(1 - pow(alpha, n));
}

Line ransac(vector<Point> point, Parameters parameter)
{
    // destructuring
    float threshold = parameter.threshold;
    cout << "Threshold : " << threshold << endl;
    int sampleSize = parameter.sampleSize;
    int samplingNumber = parameter.samplingNumber;
    int maxInliers = 0.9 * point.size();

    Line model;

    for (int index = 0; index < samplingNumber; index++)
    {
        // 무작위 3개의 점 추출
        vector<Point> randomPoints;
        while (randomPoints.size() < 3)
        {
            int randomPointIdx = rand() % point.size();
            randomPoints.push_back(point[randomPointIdx]);
        }

        // 모델 생성
        Line tmpModel = makeLine(randomPoints);

        // 모델 적합도 검사(모델과의 거리를 계산하여 ..)
        int inliers = 0;
        for (int i = 0; i < point.size(); i++)
        {
            if (getDist(tmpModel, point[i]) < threshold)
                inliers++;
        }

        if (inliers > maxInliers)
        {
            maxInliers = inliers;
            model = tmpModel;
        }
    }

    return model;
}

void saveRansacResults(const vector<Point> &points, const Line &model, const string &filename)
{
    // 현재 디렉토리 얻기
    char cwd[1024];
    string filePath;
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
    {
        filePath = string(cwd) + "/" + filename;
    }
    else
    {
        cerr << "Error: Unable to get current working directory" << endl;
        return;
    }

    // 파일 열기 (쓰기 모드)
    ofstream csv_file(filePath);
    if (!csv_file.is_open())
    {
        cerr << "Error: Unable to open file " << filePath << endl;
        return;
    }

    // 모델 계수 저장
    csv_file << "Model," << model.a << "," << model.b << "," << model.c << endl;

    csv_file.close(); // 파일 닫기
    cout << "Results saved to " << filePath << endl;
}

int main()
{
    // data formating
    vector<vector<double>> data = getData("/src/testDrive/path/data.csv");
    vector<Point> samples;
    for (int index = 0; index < data.size(); index++)
    {
        Point point;
        point.x = data[index][0];
        point.y = data[index][1];
        samples.push_back(point);
    }

    // set Parameters
    Parameters parameter;
    parameter.threshold = 0.1;
    parameter.sampleSize = 100;
    parameter.samplingNumber = getSamplingNumber(parameter.sampleSize);

    // ransac
    Line model = ransac(samples, parameter);

    // save result
    saveRansacResults(samples, model, "/src/testDrive/path/ransacResult.csv");
    return 0;
}
