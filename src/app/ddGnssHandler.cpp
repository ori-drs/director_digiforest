#include "ddGnssHandler.h"
#include <fstream> // for ofstream
#include <iomanip> // for setprecision

ddGnssHandler::ddGnssHandler(QObject* parent) : QObject(parent)
{

  // EPSG3067
  {
  const double radius = 6378137;
  const double scale_factor = 0.9996;
  const double inverse_flattening = 298.257222101;
  const double flattening = 1 / inverse_flattening;

  tm_epsg3067_ = std::make_unique<GeographicLib::TransverseMercator>(
      radius, flattening, scale_factor);
  }

  // EPSG25833
  {
  const double radius = 6378137;
  const double scale_factor = 0.9996;
  const double inverse_flattening = 298.257222101;
  const double flattening = 1 / inverse_flattening;

  tm_epsg25833_ = std::make_unique<GeographicLib::TransverseMercator>(
      radius, flattening, scale_factor);
  }

}

/// Coordinate system defined here: https://epsg.io/3067
/// Note: It defines easting and northing different from the usual convention,
/// i.e., [easting, northing, altitude]
QVector<double> ddGnssHandler::convertWGS84toEPSG3067(const double lat, const double lon) const {
  QVector<double> values; // first elem easting, second northing
  double easting, northing;                                       
  // Define constants from: https://epsg.io/3067
  const double ref_lat = 0; // deg
  const double ref_lon = 27; // deg

  const double false_easting = 500000; // m
  const double false_northing = 0; // m

  // const double radius = 6378137;
  // const double scale_factor = 0.9996;
  // const double inverse_flattening = 298.257222101;
  // const double flattening = 1 / inverse_flattening;
  // GeographicLib::TransverseMercator tm(radius, flattening, scale_factor);

  tm_epsg3067_->Forward(ref_lon, lat, lon, easting, northing);

  // Apply false_easting and false_northing
  easting += false_easting;
  northing += false_northing;
  
  values.push_back(easting);
  values.push_back(northing);
  return values;
}

QVector<double> ddGnssHandler::convertWGS84toEPSG25833(const double lat, const double lon) const {
  QVector<double> values; // first elem easting, second northing
  double easting, northing;
  // Define constants from: https://epsg.io/25833
  const double ref_lat = 0; // deg
  const double ref_lon = 15; // deg

  const double false_easting = 500000; // m
  const double false_northing = 0; // m

  // const double radius = 6378137;
  // const double scale_factor = 0.9996;
  // const double inverse_flattening = 298.257222101;
  // const double flattening = 1 / inverse_flattening;
  // GeographicLib::TransverseMercator tm(radius, flattening, scale_factor);

  tm_epsg25833_->Forward(ref_lon, lat, lon, easting, northing);

  // Apply false_easting and false_northing
  easting += false_easting;
  northing += false_northing;
  
  values.push_back(easting);
  values.push_back(northing);
  return values;
}
