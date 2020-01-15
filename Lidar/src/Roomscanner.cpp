#include <Lidar/Roomscanner.h>
#include <math.h>
#include <iostream>

Roomscanner::Roomscanner(Lidar lidar_, std::vector<Room2D> rooms_) :
    lidar(lidar_), rooms(rooms_) {}

void Roomscanner::scanRooms(const Vector3& state, const double sensor_heading,
         FullLidarMeasurement& ms) {
  ms.lidar_state = state;
  ms.measurements.clear();
  Vector2 pos = state.topRows(2);
  
  const int mps{lidar.getMeasurementsPerCycle()};
  ms.measurements.reserve(mps); // we know exactly how many measurements we make
  double global_heading = 0.;

  for (int i=0; i < mps; i++) {
    // the rotation of the sensor for this measurement, i.e. at what angle
    // from the lidar's local coordinate frame the current ray is shot at
    double local_ray_angle = i * ((2 * PI) / mps);
    
    // the orientation of the ray shot in global coordinate frame, needed
    // because the room geometry is represented in the global frame
    double lidar_global_heading = i * ((2 * PI) / mps) + state(2);
    
    Point2 end_of_ray(lidar.getRange() * cos(lidar_global_heading),
                      lidar.getRange() * sin(lidar_global_heading));
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
    // finally, add the measurement made. we store the measurements with
    // the lidar local angles
    if (has_intersection) {
      ms.measurements.push_back(Measurement(min_dist, local_ray_angle));
    } else {
      ms.measurements.push_back(Measurement(-1., local_ray_angle));
    }
  }
}

double Roomscanner::computeDistanceRayToWall(const Line& ray, const Line& wall) {
  // directional vector for ray: 
  Vector2 ray_vector = ray.p2 - ray.p1;
  Vector2 wall_vector = wall.p2 - wall.p1;
  Matrix2 A;
  A.col(0) = ray_vector;
  A.col(1) = -wall_vector;

  Vector2 b = wall.p1 - ray.p1; // right hand side of the LSE
  Vector2 x = A.colPivHouseholderQr().solve(b);
  
  // we have an intersection if the solution to above LSE is a vector
  // where both entries lie in [0,1]
  if (x(0) > 0. && x(0) <= 1. &&
      x(1) >= 0. && x(1) <= 1.) {
    // we now have to find the actual intersection point
    Point2 intersection_point = wall.p1 + x(1) * wall_vector;
    return (intersection_point - ray.p1).norm();
  } else {
    return -1.;
  }
}