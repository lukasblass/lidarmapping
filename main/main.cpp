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
void writingToFileFunction(std::vector<Measurement> ms, std::vector<Vector3> car_positions) {
  std::ofstream outstream;
  outstream.open("../Plotting/data.txt");
  for (const Vector3& pos : car_positions) {
    outstream << pos(0) << " " << pos(1) << std::endl;
  }
  outstream << "walls"<< std::endl;
  for (const Measurement& m : ms) {
    if (m.distance >= 0) {
      Point2 pt(car_positions[0](0) + m.distance * cos(m.angle), 
                car_positions[0](0) + m.distance * sin(m.angle));
      outstream << pt(0) << " " << pt(1) << std::endl;
    }
  }
  outstream.close();
}

int main() {
  std::thread plotting_thread((PlottingThread()));

  // setting up the car
  Lidar lidar(100, 8);
  Vector3 initial_state(0.5, 0.5, 0.);
  RoboCar car(lidar, initial_state);

  // storing where the car went
  std::vector<Vector3> car_states;
  car_states.push_back(initial_state);
  
  // seting up the geometry
  std::vector<Eigen::Vector2d> corners;
  corners.push_back(Vector2(5.,3.));
  corners.push_back(Vector2(5.,5.));
  corners.push_back(Vector2(0.,5.));
  corners.push_back(Vector2(0.,0.));
  corners.push_back(Vector2(5.,0.));
  corners.push_back(Vector2(5.,2.));
  
  // let the car observe its environment
  Roomscanner scanner(car.lidar, {Room2D(corners)});
  std::vector<Measurement> ms;
  scanner.scanRooms(car.getPosition(), 0., ms);
  
  car.applyAcceleration(Vector2(1., 3.), 0.2);
  car_states.push_back(car.getState());
  // now let's move the car
  for (int i=0; i<100; i++) {
    car.applyAcceleration(Vector2(0., 0.), 0.2);
    car_states.push_back(car.getState());
  }

  std::thread writeToFile(writingToFileFunction, ms, car_states);
  writeToFile.join();
  

  plotting_thread.join();
  return 1;
}