#include <stdlib.h>
#include <vector>
#include <Eigen/Dense>


#define PI 3.14159265359

struct Measurement {
  Measurement(double d, double a) : 
    distance(d), angle(a) {}
   double distance;
   double angle;
}; 

class Lidar {
  public: 
  Lidar(int measurements_) : measurements_per_cycle(measurements_) {
    for (int i = 0; i<3; i++) {
      for (int j = 0; j<3; j++) {
        matrix(i,j) = i + j;
      }
    }
  }

  std::vector<Measurement> scanACircle(const int radius);
    
  Eigen::Matrix3d matrix;

  private:
  int measurements_per_cycle = 360;
};