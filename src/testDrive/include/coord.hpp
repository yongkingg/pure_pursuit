#ifndef COORD
#define COORD

#include <string>
#include <iostream>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Constants.hpp>

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
float ref_UTM[3] = {13.5316, 1100.05, 0.00000};
string zone = "52N";

WGS84 utmToWgs84(double utm_x, double utm_y)
{
    int zone_num = std::stoi(zone.substr(0, zone.size() - 1));
    bool is_north = (zone.back() == 'N' || zone.back() == 'n');
    double lat, lon;
    GeographicLib::UTMUPS::Reverse(zone_num, is_north, utm_x, utm_y, lat, lon);

    WGS84 wgs;
    wgs.latitude = lat;
    wgs.longitude = lon;

    return wgs;
}

ENU wgs84ToENU(WGS84 wgs84)
{
    double X, Y, Z, dx, dy, dz, latitude, longitude, h;
    double ref_latitude, ref_longitude, ref_h;
    double phi, lambda, N;

    WGS84 ref_WGS = utmToWgs84(ref_UTM[0], ref_UTM[1]);
    double ref_phi = sqrt(1 - e2 * pow(sin(degToRad(ref_WGS.latitude)), 2));
    double ref_q = (a / ref_phi + ref_WGS.altitude) * cos(degToRad(ref_WGS.latitude));
    double ref_x = ref_q * cos(degToRad(ref_WGS.longitude));
    double ref_y = ref_q * sin(degToRad(ref_WGS.longitude));
    double ref_z = ((a * (1 - e2) / ref_phi) + ref_WGS.altitude) * sin(degToRad(ref_WGS.latitude));

    ENU enu;

    // WGS 변수 선언
    latitude = wgs84.latitude;
    longitude = wgs84.longitude;
    h = wgs84.altitude;
    ref_latitude = ref_WGS.latitude;
    ref_longitude = ref_WGS.longitude;
    ref_h = ref_WGS.altitude;

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

ENU utmToEnu(double utm_x, double utm_y)
{
    double offset_utm[2] = {302459.942, 4122635.537};
    WGS84 wgs84 = utmToWgs84(utm_x + offset_utm[0], utm_y + offset_utm[1]);
    return wgs84ToENU(wgs84);
}

// UTM enuToUTM(ENU enu, vector<double> offset)
// {
//     double offset_utm[2] = {302459.942, 4122635.537};
//     WGS84 ref_WGS = utmToWgs84(ref_UTM[0], ref_UTM[1]);
//     GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
//     GeographicLib::LocalCartesian local(ref_WGS.latitude, ref_WGS.longitude, ref_WGS.altitude, earth);
//     double lat, lon, alt;
//     local.Reverse(enu.East, enu.North, enu.Up, lat, lon, alt);

//     double easting, northing;
//     int zone;
//     bool northHemisphere;
//     GeographicLib::UTMUPS::Forward(lat, lon, zone, northHemisphere, easting, northing);

//     UTM utm;
//     // cout << "변환 과정 easting : " << easting << endl;
//     // cout << "변환 과정 northing : " << northing << endl;
//     utm.utm_x = easting - offset[0];
//     utm.utm_y = northing - offset[1];
//     // cout << "변환 이후 utm x : " << utm.utm_x << endl;
//     // cout << "변환 이후 utm y : " << utm.utm_y << endl;

//     return utm;
// }

UTM enuToUTM(ENU enu)
{
    double offset_utm[2] = {302459.942, 4122635.537};
    WGS84 ref_WGS;
    ref_WGS.latitude = 37.23923851363698;
    ref_WGS.longitude = 126.7731608619718;
    GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    GeographicLib::LocalCartesian local(ref_WGS.latitude, ref_WGS.longitude, ref_WGS.altitude, earth);
    double lat, lon, alt;
    local.Reverse(enu.East, enu.North, enu.Up, lat, lon, alt);
    // cout << "REF LAT : " << ref_WGS.latitude << "REF LON : " << ref_WGS.longitude << endl;
    // cout << "EAST : " << enu.East << " NORTH : " << enu.North << endl;
    // cout << "LAT : " << lat << " LON : " << lon << endl;

    double easting, northing;
    int zone;
    bool northHemisphere;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northHemisphere, easting, northing);

    UTM utm;
    // cout << "변환 과정 easting : " << easting << endl;
    // cout << "변환 과정 northing : " << northing << endl;
    utm.utm_x = easting - offset_utm[0];
    utm.utm_y = northing - offset_utm[1];
    // cout << "변환 이후 utm x : " << utm.utm_x << endl;
    // cout << "변환 이후 utm y : " << utm.utm_y << endl;

    // double offset_utm[2] = {302459.942, 4122635.537};
    // WGS84 ref_WGS = utmToWgs84(ref_UTM[0], ref_UTM[1]);
    // GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
    // GeographicLib::LocalCartesian local(ref_WGS.latitude, ref_WGS.longitude, ref_WGS.altitude, earth);
    // double lat, lon, alt;
    // local.Reverse(enu.East, enu.North, enu.Up, lat, lon, alt);

    // double easting, northing;
    // int zone;
    // bool northHemisphere;
    // GeographicLib::UTMUPS::Forward(lat, lon, zone, northHemisphere, easting, northing);

    // UTM utm;
    // // cout << "변환 과정 easting : " << easting << endl;
    // // cout << "변환 과정 northing : " << northing << endl;
    // utm.utm_x = easting - offset_utm[0];
    // utm.utm_y = northing - offset_utm[1];
    // // cout << "변환 이후 utm x : " << utm.utm_x << endl;
    // // cout << "변환 이후 utm y : " << utm.utm_y << endl;

    return utm;
}

#endif