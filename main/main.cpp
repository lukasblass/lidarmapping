#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <Lidar/Lidar.h>
#include <Lidar/Roomscanner.h>
#include <types/types.h>

int main() {
  Lidar lidar(200, 8);

  std::vector<Eigen::Vector2d> corners;
  corners.push_back(Vector2(0.,0.));
  corners.push_back(Vector2(1.,0.));
  corners.push_back(Vector2(1.,1.));
  corners.push_back(Vector2(0.,1.));
  corners.push_back(Vector2(0.,0.));
  Room2D room(corners);

  Roomscanner scanner(lidar, {room});
  
  //std::vector<Measurement> ms = scanner.lidar.scanACircle(4);
  
  std::vector<Measurement> ms;
  Vector2 pos(0.5,0.5);
  scanner.scanRooms(pos, 0., ms);
  for (const Measurement& m : ms) {
    std::cout << m.distance << " " << m.angle << std::endl;
  }

  std::ofstream outstream;
  outstream.open("../Plotting/data.txt");
  for (const Measurement& m : ms) {
    if (m.distance >= 0) {
      Point2 pt(pos(0) + m.distance * cos(m.angle), 
                pos(0) + m.distance * sin(m.angle));
      outstream << pt(0) << " " << pt(1) << std::endl;
    }
  }
  outstream.close();
}