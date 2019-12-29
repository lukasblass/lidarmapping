#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Lidar/Lidar.h>
#include <Lidar/Roomscanner.h>
#include <types/types.h>
#include <Robot/RoboCar.h>
// for threads
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <Plotting/plotting.h>

// converts the measurements to cartesian space and writes them 
// to file
void writingToFileFunction(std::vector<Measurement> ms, Vector2 pos) {
  std::ofstream outstream;
  outstream.open("../Plotting/data.txt");
  outstream << pos(0) << " " << pos(1) << std::endl;
  outstream << 0.6 << " " << 0.6 << std::endl;
  outstream << "walls"<< std::endl;
  for (const Measurement& m : ms) {
    if (m.distance >= 0) {
      Point2 pt(pos(0) + m.distance * cos(m.angle), 
                pos(0) + m.distance * sin(m.angle));
      outstream << pt(0) << " " << pt(1) << std::endl;
    }
  }
  outstream.close();
}

int main() {
  Lidar lidar(360, 8);

  std::vector<Eigen::Vector2d> corners;
  corners.push_back(Vector2(0.,0.));
  corners.push_back(Vector2(1.,0.));
  corners.push_back(Vector2(1.,1.));
  corners.push_back(Vector2(0.,1.));
  corners.push_back(Vector2(0.,0.5));
  
  std::vector<Eigen::Vector2d> corners2;
  corners2.push_back(Vector2(0.,0.5));
  corners2.push_back(Vector2(-8.,0.5));
  corners2.push_back(Vector2(-8.,0.));
  corners2.push_back(Vector2(0.,0.));
  
  Roomscanner scanner(lidar, {Room2D(corners), Room2D(corners2)});
  
  std::vector<Measurement> ms;
  Vector2 pos(0.5,0.5);
  scanner.scanRooms(pos, 0., ms);
  std::thread plotting_thread((PlottingThread()));
  std::thread writeToFile(writingToFileFunction, ms, pos);
  writeToFile.join();
  RoboCar car;

  plotting_thread.join();
  return 1;
}