#include <Lidar/Roomscanner.h>
#include <math.h>
#include <iostream>

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
    end_of_ray = end_of_ray + pos;                      
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
  // directional vector for ray: 
  Vector2 ray_vector = ray.p1 - ray.p2;
  Vector2 wall_vector = wall.p1 - wall.p2;
  Matrix2 A;
  A.col(0) = ray_vector;
  A.col(1) = -wall_vector;

  Vector2 b = ray.p1 - wall.p1; // right hand side of the LSE
  Vector2 x = A.colPivHouseholderQr().solve(b);
  
  // we have an intersection if the solution to above LSE is a vector
  // where both entries lie in [0,1]
  if (x(0) >= 0. && x(0) <= 1. &&
      x(1) >= 0. && x(1) <= 1.) {
    // we now have to find the actual intersection point
    Point2 intersection_point = ray.p1 + x(0) * ray_vector;
    return (intersection_point - ray.p1).norm();
  } else {
    return -1.;
  }
}