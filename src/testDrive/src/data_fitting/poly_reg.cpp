#include <iostream>
#include <vector>
#include <cmath>
#include <csv.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

struct PolyReg
{
    vector<double> x;
    double y;
};


vector<vector<double>> polyTransform(vector<Point> samples, int degree) {
    vector<vector<double>> result; 
    for (int i = 0; i < samples.size(); i++) {
        vector<double> transformedX;

        for (int j = 0; j <= degree; j++) { 
            transformedX.push_back(pow(samples[i].x, j));
        }

        result.push_back(transformedX);
    }

    return result;
}

vector<double> getCoefficients(vector<vector<double>> x, vector<double> y, int degree) {
    int sampleSize = x.size();

    MatrixXd X(sampleSize, degree+1);
    VectorXd Y(sampleSize);

    for (int i = 0; i < sampleSize; i++) {
        for (int j = 0; j < degree + 1; j++) {
            X(i, j) = x[i][j];
        }
        Y(i) = y[i];
    }

    VectorXd beta = (X.transpose() * X).inverse() * X.transpose() * Y;
    vector<double> result(beta.data(), beta.data() + beta.size());

    return result;
}

void savePolyResults(const vector<double> &points, const string &filename)
{
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

    ofstream csv_file(filePath);
    if (!csv_file.is_open())
    {
        cerr << "Error: Unable to open file " << filePath << endl;
        return;
    }

    // 모델 계수 저장
    for (int index = 0; index < points.size(); index++) {
        csv_file << points[index];
        if (index != points.size() - 1) {
            csv_file << " , ";
        }
    }

    csv_file.close(); // 파일 닫기
    cout << "Results saved to " << filePath << endl;
}


int main()
{
    vector<vector<double>> data = getData("/src/testDrive/path/data.csv");
    vector<Point> samples;
    for (int index = 0; index < data.size(); index++)
    {
        Point point;
        point.x = data[index][0];
        point.y = data[index][1];
        samples.push_back(point);
    }

    int degree = 3;
    vector<vector<double>> polyResultX = polyTransform(samples, degree);
    vector<double> y;
    for (const auto sample : samples) {
        y.push_back(sample.y);
    }

    vector<double> coefficents = getCoefficients(polyResultX, y, degree);

    savePolyResults(coefficents, "src/testDrive/path/polyResult.csv");
    for(const auto item : coefficents) {
        cout << item << endl;
    }
    // /home/yongkingg/Desktop/pure_pursuit/
    // /home/yongkingg/Desktop/pure_pursuit/src/testDrive/path/polyResult.csv
    // /home/yongkingg/Desktop/pure_pursuit/testDrive/path/polyResult.csv

    return 0;
}
