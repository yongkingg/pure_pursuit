#ifndef GET_CSV
#define GET_CSV

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace std;

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

#endif