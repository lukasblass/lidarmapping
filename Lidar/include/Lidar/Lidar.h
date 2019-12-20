#include <stdlib.h>
#include <vector>

#define PI 3.14159265359

struct Measurement {
    Measurement(double d, double a) : 
            distance(d), angle(a) {}
    double distance;
    double angle;
}; 

class Lidar {
    public: 
    Lidar(int measurements_) : measurements_per_cycle(measurements_) {}

    std::vector<Measurement> scanACircle(const int radius);

    private:
    int measurements_per_cycle = 360;
};