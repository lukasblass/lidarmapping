#include <stdlib.h>
#include <vector>
#include <types/types.h>

#ifndef LIDAR_H_
#define LIDAR_H_

const double PI =  3.14159265359;

struct Measurement {
  Measurement(double d, double a) : 
    distance(d), angle(a) {}
   double distance;
   double angle;
};

struct FullLidarMeasurement {
  std::vector<Measurement> measurements;
  Vector3 lidar_state; // position and orientation at measurement
};

class Lidar {
  public: 
  Lidar(int measurements_, double range_) : measurements_per_cycle(measurements_),
    range(range_), sigma(0.) {
  }

  Lidar(int measurements_, double range_, double sigma_) : measurements_per_cycle(measurements_),
    range(range_), sigma(sigma_) {
  }

  std::vector<Measurement> scanACircle(const int radius);
  int getMeasurementsPerCycle();
  double getRange();
  double getSigma();
  
  private:
  int measurements_per_cycle = 360;
  double range;
  double sigma; // std deviation of the noise
};

#endif