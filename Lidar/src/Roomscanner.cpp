#include <Lidar/Roomscanner.h>
#include <math.h>

Roomscanner::Roomscanner(Lidar lidar_, std::vector<Room2D> rooms_) :
    lidar(lidar_), rooms(rooms_) {}

void Roomscanner::scanRooms(const Point2& pos, const double sensor_heading,
         std::vector<Measurement>& measurements) {
  measurements.clear();
  const double PI = 3.14159265359;
  
  const int mps{lidar.getMeasurementsPerCycle()};
  measurements.reserve(mps); // we know exactly how many measurements we make
  double heading = 0.;

  for (int i=0; i < mps; i++) {
    double heading = i * ((2 * PI) / mps);
    Point2 end_of_ray(lidar.getRange() * cos(heading),
                       lidar.getRange() * sin(heading));
    Line ray(pos, end_of_ray);

    double min_dist = INFINITY;
    bool has_intersection = false;
    for (const Room2D& room : rooms) {
      const std::vector<Point2>& corners = room.getCorners(); 
      
      for (int j=0; j < corners.size() - 1; j++) {
        Line wall(corners[j], corners[j+1]);
        double distance = computeDistanceRayToWall(ray, wall);
        if (distance >= 0 && distance < min_dist) {
          min_dist = distance;
          has_intersection = true;
        }
      }
    }
    if (has_intersection) {
      measurements.push_back(Measurement(min_dist, heading));
    } else {
      measurements.push_back(Measurement(-1., heading));
    }
  }
}

double Roomscanner::computeDistanceRayToWall(const Line& ray, const Line& wall) {
  // first, we mirror the ray at the wall
  return 1.;
}