#ifndef MY_CSV
#define MY_CSV

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <iomanip>
using namespace std;

struct Frenet
{
    float s, d;
};

struct Point
{
    float x, y;
};

void saveFrenetToCSV(const vector<Frenet> &frenet_data, const string &file_path)
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

void saveXYToCSV(const vector<Point> &xyPath, const string &filePath)
{
    string full_path;
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
    {
        full_path = string(cwd) + "/" + filePath;
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

    for (const auto &point : xyPath)
    {
        file_stream << point.x << "," << point.y << endl;
    }

    file_stream.close();
}

vector<vector<float>> getPath(string file_path)
{
    string file;
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
    {
        file = cwd + file_path;
    }
    else
    {
        cout << "getcwd err" << endl;
    }

    ifstream csv_file(file);
    if (!csv_file.is_open())
    {
        cerr << "Error: Unable to open file " << file << endl;
        return {};
    }

    vector<vector<float>> path_data;
    string line;
    while (getline(csv_file, line))
    {
        if (line.empty())
            continue;
        stringstream line_stream(line);
        string cell;
        vector<float> point;
        while (getline(line_stream, cell, ','))
        {
            try
            {
                point.push_back(stof(cell));
            }
            catch (const std::invalid_argument &e)
            {
                cerr << "Invalid argument: " << cell << endl;
                continue;
            }
        }
        if (point.size() == 2)
        {
            path_data.push_back(point);
        }
    }
    csv_file.close();
    return path_data;
}

vector<vector<double>> getData(string file_path)
{
    string file;
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr)
    {
        file = cwd + file_path;
    }
    else
    {
        cout << "getcwd err" << endl;
    }

    ifstream csv_file(file);
    if (!csv_file.is_open())
    {
        cerr << "Error: Unable to open file " << file << endl;
        return {};
    }

    vector<vector<double>> path_data;
    string line;
    while (getline(csv_file, line))
    {
        if (line.empty())
            continue;
        stringstream line_stream(line);
        string cell;
        vector<double> point;
        while (getline(line_stream, cell, ','))
        {
            try
            {
                point.push_back(stof(cell));
            }
            catch (const std::invalid_argument &e)
            {
                cerr << "Invalid argument: " << cell << endl;
                continue;
            }
        }
        if (point.size() == 2)
        {
            path_data.push_back(point);
        }
    }
    csv_file.close();
    return path_data;
}

#endif