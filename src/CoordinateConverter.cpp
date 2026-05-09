#include "include_CoordinateConverter.hpp"

#include <cmath>
#include <stdexcept>

namespace loc {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

}  // namespace

CoordinateConverter::CoordinateConverter() : CoordinateConverter(Ellipsoid{}) {}

CoordinateConverter::CoordinateConverter(Ellipsoid ellipsoid) : ellipsoid_(ellipsoid) {
  if (ellipsoid_.semi_major_axis_m <= 0.0) {
    throw std::runtime_error("semi_major_axis_m doit etre > 0");
  }
  if (ellipsoid_.flattening <= 0.0 || ellipsoid_.flattening >= 1.0) {
    throw std::runtime_error("flattening doit etre dans ]0,1[");
  }
}

GeodeticCoordinate CoordinateConverter::EcefToGeodetic(double x_m,
                                                       double y_m,
                                                       double z_m) const {
  const double a = ellipsoid_.semi_major_axis_m;
  const double f = ellipsoid_.flattening;
  const double b = a * (1.0 - f);

  const double e2 = f * (2.0 - f);
  const double ep2 = (a * a - b * b) / (b * b);

  const double p = std::sqrt(x_m * x_m + y_m * y_m);
  const double longitude = std::atan2(y_m, x_m);

  double latitude = 0.0;
  double altitude = 0.0;

  if (p < 1e-12) {
    latitude = (z_m >= 0.0) ? (kPi * 0.5) : (-kPi * 0.5);
    altitude = std::abs(z_m) - b;
  } else {
    const double theta = std::atan2(z_m * a, p * b);
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);

    latitude = std::atan2(z_m + ep2 * b * sin_theta * sin_theta * sin_theta,
                          p - e2 * a * cos_theta * cos_theta * cos_theta);

    const double sin_lat = std::sin(latitude);
    const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
    altitude = p / std::cos(latitude) - N;
  }

  GeodeticCoordinate out;
  out.latitude_deg = latitude * kRadToDeg;
  out.longitude_deg = longitude * kRadToDeg;
  out.altitude_m = altitude;
  return out;
}

}  // namespace loc