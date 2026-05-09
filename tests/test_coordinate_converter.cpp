#include "include_CoordinateConverter.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

bool AlmostEqual(double a, double b, double eps) {
  return std::abs(a - b) <= eps;
}

void ExpectNear(double value, double expected, double eps, const char* label) {
  if (!AlmostEqual(value, expected, eps)) {
    throw std::runtime_error(std::string("Valeur inattendue pour ") + label);
  }
}

}  // namespace

int main() {
  try {
    const loc::CoordinateConverter conv;

    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);

    {
      const loc::GeodeticCoordinate geo = conv.EcefToGeodetic(a, 0.0, 0.0);
      ExpectNear(geo.latitude_deg, 0.0, 1e-9, "latitude equateur");
      ExpectNear(geo.longitude_deg, 0.0, 1e-9, "longitude equateur");
      ExpectNear(geo.altitude_m, 0.0, 1e-6, "altitude equateur");
    }

    {
      const loc::GeodeticCoordinate geo = conv.EcefToGeodetic(0.0, a, 0.0);
      ExpectNear(geo.latitude_deg, 0.0, 1e-9, "latitude Y+");
      ExpectNear(geo.longitude_deg, 90.0, 1e-9, "longitude Y+");
      ExpectNear(geo.altitude_m, 0.0, 1e-6, "altitude Y+");
    }

    {
      const loc::GeodeticCoordinate geo = conv.EcefToGeodetic(0.0, 0.0, b);
      ExpectNear(geo.latitude_deg, 90.0, 1e-9, "latitude pole nord");
      ExpectNear(geo.altitude_m, 0.0, 1e-6, "altitude pole nord");
    }

    {
      const loc::CoordinateConverter::Ellipsoid invalid{0.0, f};
      bool threw = false;
      try {
        const loc::CoordinateConverter invalid_conv(invalid);
        (void)invalid_conv;
      } catch (const std::exception&) {
        threw = true;
      }
      if (!threw) {
        throw std::runtime_error("Le constructeur doit rejeter semi_major_axis_m <= 0");
      }
    }

    std::cout << "[PASS] test_coordinate_converter" << std::endl;
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[FAIL] test_coordinate_converter: " << e.what() << std::endl;
    return 1;
  }
}