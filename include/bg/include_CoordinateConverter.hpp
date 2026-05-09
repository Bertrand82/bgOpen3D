#pragma once

namespace loc {

struct GeodeticCoordinate {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double altitude_m = 0.0;
};

class CoordinateConverter {
public:
  struct Ellipsoid {
    double semi_major_axis_m = 6378137.0;             // WGS84 a
    double flattening = 1.0 / 298.257223563;         // WGS84 f
  };

  CoordinateConverter();
  explicit CoordinateConverter(Ellipsoid ellipsoid);

  GeodeticCoordinate EcefToGeodetic(double x_m, double y_m, double z_m) const;

private:
  Ellipsoid ellipsoid_;
};

}  // namespace loc