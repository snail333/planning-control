#include <llh2utm/llh2utm.hpp>
#include <ros/ros.h>
#include <cmath>

gps_utm::gps_utm()
    : m_x(0)
    , m_y(0)
    , m_z(0)
    , m_lat(0)
    , m_lon(0)
    , m_h(0)
{
}

gps_utm::~gps_utm(){}

double gps_utm::x() const
{
  return m_x;
}

double gps_utm::y() const
{
  return m_y;
}

double gps_utm::z() const
{
  return m_z;
}

void gps_utm::set_zone(double lat, double lon){

  GeographicLib::UTMUPS::Forward(lat, lon, zone_, northp_, origin_x_, origin_y_);
  // ROS_INFO_STREAM("zone_:"<<zone_<<" north_: "<<northp_<<" origin_x: "<<origin_x_<<" origin_y_: "<<origin_y_);
}

void gps_utm::set_offset(bool offset_flag){
  use_offset_ = offset_flag;
}

void gps_utm::llh_to_xyz_geolib(double lat, double lon, double ele)
{
  m_lat = lat;
  m_lon = lon;
  m_h = ele;

  conv_llh2xyz_geolib();
}

void gps_utm::conv_llh2xyz_geolib(void){
  int zone{};
  bool northp{};
  try {
    GeographicLib::UTMUPS::Forward(m_lat, m_lon, zone, northp, m_x, m_y);
  } catch (GeographicLib::GeographicErr& e) {
    throw GeographicLib::GeographicErr(e.what());
  }
  if (zone != zone_ || northp != northp_) {
    // try to transfer to the desired zone
    double xAfterTransfer = 0;
    double yAfterTransfer = 0;
    int zoneAfterTransfer = 0;
    try {
      GeographicLib::UTMUPS::Transfer(zone, northp, m_x, m_y, zone_, northp_, xAfterTransfer,
                                      yAfterTransfer, zoneAfterTransfer);
    } catch (GeographicLib::GeographicErr& e) {
      throw GeographicLib::GeographicErr(e.what());
    }

    if (zoneAfterTransfer != zone_) {
      throw GeographicLib::GeographicErr("You have left the padding area of the UTM zone!");
    }
    m_x = xAfterTransfer;
    m_y = yAfterTransfer;
  }

  if (use_offset_) 
  {
    m_x -= origin_x_;
    m_y -= origin_y_;
  }

}
