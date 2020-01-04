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
void writingToFileFunction(std::vector<Measurement> ms, std::vector<Vector3> car_states,
        const std::pair<double,double> car_dimensions) {
  std::ofstream outstream;
  outstream.open("../Plotting/data.txt");
  double w = car_dimensions.first / 2;
  double l = car_dimensions.second;
  for (const Vector3& pos : car_states) {
    // build the transformation matrix mapping from the cars to the intertial coordinate frame
    const double theta = pos(2);
    Matrix3 transformation;
    transformation << cos(theta), -sin(theta), pos(0), 
                      sin(theta),  cos(theta), pos(1),
                      0,           0,          1;
    Vector3 c1, c2, c3, c4; // the corners of the car represented in the cars coordinate frame
    c1 = Vector3(l, w, 1);
    c2 = Vector3(0, w, 1);
    c3 = Vector3(0, -w, 1);
    c4 = Vector3(l, -w, 1);
    // now transform the corners of the car to the intertial coordinate frame
    c1 = transformation * c1;
    c2 = transformation * c2;
    c3 = transformation * c3;
    c4 = transformation * c4;
    // and write the corners to file
    outstream <<  c1(0) << " " << c1(1) << " " <<
                  c2(0) << " " << c2(1) << " " <<
                  c3(0) << " " << c3(1) << " " <<
                  c4(0) << " " << c4(1) << std::endl;
  }
  outstream << "walls"<< std::endl;
  for (const Measurement& m : ms) {
    if (m.distance >= 0) {
      Point2 pt(car_states[0](0) + m.distance * cos(m.angle), 
                car_states[0](0) + m.distance * sin(m.angle));
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
  // now let's move the car
  for (int i=0; i<50; i++) {
    car.applyControlInput(Vector2(.2, .3), 0.1);
    if (i % 5 == 0) {
      car_states.push_back(car.getState());
    } 
  }
  for (int i=0; i<50; i++) {
    car.applyControlInput(Vector2(.2, -.3), 0.1);
    if (i % 5 == 0) {
      car_states.push_back(car.getState());
    } 
  }
  std::thread writeToFile(writingToFileFunction, ms, car_states, car.CAR_DIMENSIONS);
  writeToFile.join();
  
  plotting_thread.join();
  return 1;
}