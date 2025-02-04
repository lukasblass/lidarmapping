
#include <Lidar/Lidar.h>

std::vector<Measurement> Lidar::scanACircle(const int radius) {
  std::vector<Measurement> out;

  for (int i=0; i<measurements_per_cycle; i++) {
    double angle = i * ((2 * PI) / measurements_per_cycle);
    out.push_back(Measurement(radius, angle));
  }
  return out;
}

int Lidar::getMeasurementsPerCycle() {
  return measurements_per_cycle;
}

double Lidar::getRange() {
  return range;
}

double Lidar::getSigma() {
  return sigma;
}