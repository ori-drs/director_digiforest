#pragma once

#include <memory>
#include <iostream>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include "ddAppConfigure.h"

#include <QObject>
#include <QVector>


///
/// There are three main coordinate frames used in this class.
/// LLA:
/// - Latitude, longitude, altitude (global Earth-centered coordinate system)
/// - This is what is measured by the GNSS sensor.
///
/// ENU:
/// - East, north, up (global Earth-centered coordinate system)
/// - An Euclidean coordinate system
///
/// Map:
/// - VILENS SLAM local coordinate system
///
class DD_APP_EXPORT ddGnssHandler : public QObject{
  Q_OBJECT
 public:

  ddGnssHandler(QObject* parent=0);
  ~ddGnssHandler() = default;

  // LLA <> EPSG-3067 (Finland coordinate system)
  // return easting and northing, in that order
  QVector<double> convertWGS84toEPSG3067(const double lat, const double lon) const;

  // LLA <> EPSG-25833 (Europe between 12E and 18E, including Germany)
  // return easting and northing, in that order
  QVector<double> convertWGS84toEPSG25833(const double lat, const double lon) const;

 protected:

  // Keep objects to speed up conversions to other coordinate systems
  std::unique_ptr<GeographicLib::TransverseMercator> tm_epsg25833_;
  std::unique_ptr<GeographicLib::TransverseMercator> tm_epsg3067_;
};

