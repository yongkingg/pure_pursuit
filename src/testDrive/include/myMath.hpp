#ifndef MY_MATH
#define MY_MATH

#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

float getDistance(float x1, float x2, float y1, float y2)
{
    return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}

float getDistance(float x1, float y1)
{
    return sqrt(pow(x1, 2) + pow(y1, 2));
}

#endif