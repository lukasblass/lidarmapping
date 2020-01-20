#include <stdlib.h>
#include <vector>
#include <random>
#include "Lidar.h"
#include <Roommap/Room2D.h>
#include <types/types.h>
// for threads
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>

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

  private:
  // sets up an LSE with 2 equations and 2 unknowns by setting
  // ray.p1 + a * (ray.p2 - ray.p1) = wall.p1 + b * (wall.p2 - wall.p1)
  // if the LSE has a solution with a,b in [0,1] the lines intersect
  // and we can compute directly the distance from ray.p1 to the intersection point
  double computeDistanceRayToWall(const Line& ray, const Line& wall);

  /**
   * \brief takes a full lidar measurement and adds the information
   * to the observed geometry
   */
  void updateObservedGeometry(const FullLidarMeasurement& fms);

  void scanRoomsThread(const Vector3& state, const double I_theta_start,
     FullLidarMeasurement &ms, const int start_index, const int end_index);
  
  void mapMeasurementsToGrid(const FullLidarMeasurement& fms, int start_index,
      int end_index);
  
  void updateCleanedGrid(int start_i, int start_j, int sz);

  public:
  Lidar lidar;
  std::vector<Room2D> rooms;
  // aggregated scans with cleaned geometry, Points stored in inertial frame
  std::vector<Point2> observed_geometry;
  Eigen::MatrixXi grid;
  Eigen::MatrixXi cleaned_grid;
  
  // for adding lidar noise
  std::normal_distribution<double> distribution_lidar;
  std::mt19937 gen;
  bool add_lidar_noise;
  
};

#endif