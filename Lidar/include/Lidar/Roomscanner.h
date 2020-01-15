#include <stdlib.h>
#include <vector>
#include "Lidar.h"
#include <Roommap/Room2D.h>
#include <types/types.h>

#ifndef ROOMSCANNER_H_
#define ROOMSCANNER_H_

struct Line {
  Line(Point2 p1_, Point2 p2_) : p1(p1_), p2(p2_) {}
  Point2 p1;
  Point2 p2;
};

class Roomscanner {
  public:
  Roomscanner(Lidar lidar_, std::vector<Room2D> rooms_);
  
  /**
  * \brief computes the measurements for the lidar at the current
    position in the room
    \param @pos [in] the current position of the lidar
    \param @measurements [out] a vector measurements
  */
  void scanRooms(const Vector3& state, const double heading, 
         FullLidarMeasurement& measurement);

  // sets up an LSE with 2 equations and 2 unknowns by setting
  // ray.p1 + a * (ray.p2 - ray.p1) = wall.p1 + b * (wall.p2 - wall.p1)
  // if the LSE has a solution with a,b in [0,1] the lines intersect
  // and we can compute directly the distance from ray.p1 to the intersection point
  double computeDistanceRayToWall(const Line& ray, const Line& wall);

  public:
  Lidar lidar;
  std::vector<Room2D> rooms;
};

#endif