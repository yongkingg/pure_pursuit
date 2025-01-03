#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

struct Point
{
    float x, y;
};

// y = ax + b
// ax - y + b = 0
struct Line
{
    float a, b;
};

float getDist(Line line, float x1, float y1)
{
    return abs(line.a * x1 - y1 + line.b) / sqrt(pow(line.a, 2) + 1);
}

void ransac(Point point)
{
}

int main()
{
    return 0;
}