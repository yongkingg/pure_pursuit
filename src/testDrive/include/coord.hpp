#ifndef COORD
#define COORD

#include <string>
#include <iostream>
#include <cmath>

using namespace std;

// WGS 84 타원체 상수
const double a = 6378137.0;         // 적도 반경 (m)
const double f = 1 / 298.257223563; // 편평률
const double e2 = 2 * f - pow(f, 2);

struct WGS84
{
    double latitude;
    double longitude;
    double altitude;
};

struct UTM
{
    double timestamp;
    double utm_x;
    double utm_y;
    string zone;
};

struct ENU
{
    double East;
    double North;
    double Up;
};

struct NED
{
    double North;
    double East;
    double Down;
};

struct Euler
{
    double roll;
    double pitch;
    double yaw;
};

struct ECEF
{
    double X;
    double Y;
    double Z;
};

struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

// =========================== METHOD =========================== //
double radToDeg(double rad)
{
    return rad / M_PI * 180;
}
double degToRad(double deg)
{
    return deg / 180 * M_PI;
}

// ========================= PARAMETERS ========================= //
float ref_WGS[3] = {0,0,0};
// float ref_WGS[3] = {37.238838359501933, 126.772902206454901, 0.000000000000000};

double ref_phi = sqrt(1 - e2 * pow(sin(degToRad(ref_WGS[0])), 2));
double ref_q = (a / ref_phi + ref_WGS[2]) * cos(degToRad(ref_WGS[0]));
double ref_x = ref_q * cos(degToRad(ref_WGS[1]));
double ref_y = ref_q * sin(degToRad(ref_WGS[1]));
double ref_z = ((a * (1 - e2) / ref_phi) + ref_WGS[2]) * sin(degToRad(ref_WGS[0]));
string zone = "52N";

ENU wgs84ToENU(WGS84 wgs84)
{
    double X, Y, Z, dx, dy, dz, latitude, longitude, h;
    double ref_latitude, ref_longitude, ref_h;
    double phi, lambda, N;
    ENU enu;

    // WGS 변수 선언
    latitude = wgs84.latitude;
    longitude = wgs84.longitude;
    h = wgs84.altitude;
    ref_latitude = ref_WGS[0];
    ref_longitude = ref_WGS[1];
    ref_h = ref_WGS[2];

    // rad로 변환
    phi = degToRad(latitude);
    lambda = degToRad(longitude);
    N = a / sqrt(1 - e2 * sin(phi) * sin(phi));

    // ECEF 좌표
    X = (N + h) * cos(phi) * cos(lambda);
    Y = (N + h) * cos(phi) * sin(lambda);
    Z = ((1 - e2) * N + h) * sin(phi);

    dx = X - ref_x;
    dy = Y - ref_y;
    dz = Z - ref_z;

    // ENU 변환
    enu.East = -sin(degToRad(ref_longitude)) * dx + cos(degToRad(ref_longitude)) * dy;
    enu.North = -sin(degToRad(ref_latitude)) * cos(degToRad(ref_longitude)) * dx - sin(degToRad(ref_latitude)) * sin(degToRad(ref_longitude)) * dy + cos(degToRad(ref_latitude)) * dz;
    enu.Up = cos(degToRad(ref_latitude)) * cos(degToRad(ref_longitude)) * dx + cos(degToRad(ref_latitude)) * sin(degToRad(ref_longitude)) * dy + sin(degToRad(ref_latitude)) * dz;

    return enu;
}

Euler quatToEnuEuler(Quaternion q)
{
    Euler ned_euler;
    ned_euler.roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (pow(q.x, 2) + pow(q.y, 2)));
    ned_euler.pitch = asin(2 * (q.w * q.y - q.z * q.x));
    ned_euler.yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (pow(q.y, 2) + pow(q.z, 2)));

    Euler enu_euler;
    enu_euler.roll = ned_euler.roll;
    enu_euler.pitch = ned_euler.pitch;
    enu_euler.yaw = M_PI / 2 - ned_euler.yaw;

    return enu_euler;
}


// UTM -> WGS84 변환 함수
void utmToWgs84(double utm_x, double utm_y, double &lat, double &lon)
{
    int zone_num = std::stoi(zone.substr(0, zone.size() - 1)); 
    bool is_north = (zone.back() == 'N'); 
    // UTM -> WGS84 변환
    GeographicLib::UTMUPS::Reverse(zone_num, is_north, utm_x, utm_y, lat, lon);
}

ENU utmToEnu(double utm_x, double utm_y)
{
    double lat, lon, alt = 0.0;

    utmToWgs84(utm_x, utm_y, lat, lon);

    WGS84 wgs84;
    wgs84.latitude = lat;
    wgs84.longitude = lon;
    wgs84.altitude = alt;
    return wgs84ToENU(wgs84);
}
// =========================== METHOD =========================== //

#endif