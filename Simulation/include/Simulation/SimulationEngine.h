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


class Simulator {
  public:
  Simulator();

  void run();

  private:
  Lidar lidar;
  RoboCar car;
  std::vector<Room2D> rooms;
};