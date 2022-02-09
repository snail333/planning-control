/*
 *
 * 
 * 
 */

#ifndef LLH2UTM_HPP
#define LLH2UTM_HPP
#include <GeographicLib/UTMUPS.hpp>

class gps_utm
{
private:
  double m_x;  // m
  double m_y;  // m
  double m_z;  // m

  double m_lat;  // latitude
  double m_lon;  // longitude
  double m_h;

  int zone_; // utm zone
  bool northp_;
  double origin_x_, origin_y_;
  bool use_offset_ = false;

public:
  gps_utm();
  ~gps_utm();
  double x() const;
  double y() const;
  double z() const;

  void set_zone(double lat, double lon);
  void set_plane(double lat, double lon);
  void set_plane(int num);
  void set_offset(bool offset_flag);
  void set_xyz(double cx, double cy, double cz);

  // set llh in radians
  void set_llh(double lat, double lon, double h);

  // set llh in nmea degrees
  void set_llh_nmea_degrees(double latd, double lond, double h);

  void llh_to_xyz(double lat, double lon, double ele);

  void conv_llh2xyz(void);
  void conv_xyz2llh(void);
  void conv_llh2xyz_geolib(void);
  void llh_to_xyz_geolib(double lat, double lon, double ele);
};

#endif  // LLH2UTM_HPP
