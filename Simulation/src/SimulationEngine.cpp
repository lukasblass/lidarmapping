#include <Simulation/SimulationEngine.h>

// TODO create a config file to store parameters for 
// the different objects instantiated here
Simulator::Simulator() {
  Lidar lidar(100, 8., 0.02);
  car = new RoboCar(lidar, Vector3(0.5, 3.5, 0.));
  
  // seting up the geometry
  // TODO also read this from file
  std::vector<Room2D> rooms;
  std::vector<Eigen::Vector2d> room;
  room.push_back(Vector2(5.,6.));
  room.push_back(Vector2(5.,8.));
  room.push_back(Vector2(0.,8.));
  room.push_back(Vector2(0.,3.));
  room.push_back(Vector2(5.,3.));
  room.push_back(Vector2(5.,5.));
  rooms.push_back(Room2D(room));

  room.clear();
  room.push_back(Vector2(5.,5.));
  room.push_back(Vector2(7.,5.));
  room.push_back(Vector2(7.,2.));
  room.push_back(Vector2(9.,2.));
  room.push_back(Vector2(9.,6.));
  room.push_back(Vector2(5.,6.));
  rooms.push_back(room);

  roomscanner = new Roomscanner(car->lidar, rooms);
}

void realTimeWriteToFile(const Eigen::MatrixXi& grid, Vector3 car_state,
                        Vector3 est_car_state,
                        const std::pair<double,double> car_dimensions) {
  std::string out;
  double w = car_dimensions.first / 2;
  double l = car_dimensions.second;

  // build the transformation matrix mapping from the cars to the intertial coordinate frame
  double theta = car_state(2);
  Matrix3 transformation;
  transformation << cos(theta), -sin(theta), car_state(0), 
                    sin(theta),  cos(theta), car_state(1),
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
  out += std::to_string(c1(0)) + " "  +  std::to_string(c1(1)) + " " +
         std::to_string(c2(0)) + " "  +  std::to_string(c2(1)) + " " +
         std::to_string(c3(0)) + " "  +  std::to_string(c3(1)) + " " +
         std::to_string(c4(0)) + " "  +  std::to_string(c4(1)) + "\n";
  
  out += "car_estimations\n";
  theta = est_car_state(2);
  transformation << cos(theta), -sin(theta), est_car_state(0), 
                    sin(theta),  cos(theta), est_car_state(1),
                    0,           0,          1;
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
  out += std::to_string(c1(0)) + " "  +  std::to_string(c1(1)) + " " +
         std::to_string(c2(0)) + " "  +  std::to_string(c2(1)) + " " +
         std::to_string(c3(0)) + " "  +  std::to_string(c3(1)) + " " +
         std::to_string(c4(0)) + " "  +  std::to_string(c4(1)) + "\n";                

  out += "walls\n";
  bool output = false;
  for (int i=0; i<grid.rows(); i++) {
    for (int j=0; j<grid.cols(); j++) {
      if (grid(i,j) == 1) {
        out += std::to_string(i / 10. + 0.05) + " " + std::to_string(j / 10. + 0.05) + "\n";
        output = true;
      }
    }
  }
  if (!output) {
    out += std::to_string(2.5) + std::to_string(2.5) + "\n";
  }
  std::ofstream outstream;
  outstream.open("../Plotting/data.txt");
  outstream << out;
  outstream.close();
}

void Simulator::advanceInTime(const Vector2& signal, std::vector<std::thread>& threads,
   const double dt) {
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  car->applyControlInput(signal, dt);
  roomscanner->scanRooms(car->getActualState());
  threads.push_back(std::thread(realTimeWriteToFile, std::ref(roomscanner->cleaned_grid), car->getActualState(),
  car->getMeasuredState(), car->CAR_DIMENSIONS));
  
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  const int64_t duration = std::chrono::duration_cast<
        std::chrono::milliseconds>(end - begin).count();
  int dt_in_ms = int(dt * 100);
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_in_ms - 5 - duration));
}


void Simulator::run() {

  std::thread plotting_thread((PlottingThread()));
  
  FullLidarMeasurement full_measurement;
  roomscanner->scanRooms(car->getActualState());
  std::chrono::steady_clock::time_point g_begin = std::chrono::steady_clock::now();
  std::vector<std::thread> threads;
  // now let's move the car
  for (int i=0; i<50; i++) {
    advanceInTime(Vector2(.2, 0.3), threads, dt);
  }
  for (int i=0; i<50; i++) {
    advanceInTime(Vector2(.2, -0.3), threads, dt);
  }
  for (int i=0; i<50; i++) {
    advanceInTime(Vector2(.5, 0.), threads, dt);
  }
  for (auto& t : threads) {
    t.join();
  }

  plotting_thread.join();
}