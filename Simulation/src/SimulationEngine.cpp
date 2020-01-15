#include <Simulation/SimulationEngine.h>

// TODO create a config file to store parameters for 
// the different objects instantiated here
Simulator::Simulator() : lidar(100, 8),
      car(lidar, Vector3(0.5, 0.5, 0.)) {

  // seting up the geometry
  // TODO also read this from file
  std::vector<Eigen::Vector2d> room;
  room.push_back(Vector2(5.,3.));
  room.push_back(Vector2(5.,5.));
  room.push_back(Vector2(0.,5.));
  room.push_back(Vector2(0.,0.));
  room.push_back(Vector2(5.,0.));
  room.push_back(Vector2(5.,2.));

  rooms.push_back(Room2D(room));
}

// converts the measurements to cartesian space and writes them 
// to file
void writingToFileFunction(std::vector<FullLidarMeasurement> fms, std::vector<Vector3> car_states,
        std::vector<Vector3> measured_car_states,
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
    // and write thge corners to file
    outstream <<  c1(0) << " " << c1(1) << " " <<
                  c2(0) << " " << c2(1) << " " <<
                  c3(0) << " " << c3(1) << " " <<
                  c4(0) << " " << c4(1) << std::endl;
  }
  outstream << "car_estimations" << std::endl;
  for (const Vector3& state : measured_car_states) {
    // build the transformation matrix mapping from the cars to the intertial coordinate frame
    const double theta = state(2);
    Matrix3 transformation;
    transformation << cos(theta), -sin(theta), state(0), 
                      sin(theta),  cos(theta), state(1),
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
    // and write thge corners to file
    outstream <<  c1(0) << " " << c1(1) << " " <<
                  c2(0) << " " << c2(1) << " " <<
                  c3(0) << " " << c3(1) << " " <<
                  c4(0) << " " << c4(1) << std::endl;
  }

  outstream << "walls"<< std::endl;
  for (const FullLidarMeasurement& ms : fms) {
    double theta = ms.lidar_state(2);
    double x = ms.lidar_state(0);
    double y = ms.lidar_state(1);
    
    Matrix3 T;
    T <<  cos(theta), -sin(theta), x,
          sin(theta),  cos(theta), y,
          0,           0,          1; 
    for (const Measurement& m : ms.measurements) {
     if (m.distance >= 0) {
      Point3 pt(m.distance * cos(m.angle), 
                m.distance * sin(m.angle), 1);
      pt = T * pt;
      outstream << pt(0) << " " << pt(1) << std::endl;
      }
    }
  }
  outstream.close();
}

void Simulator::run() {

  std::thread plotting_thread((PlottingThread()));

  // storing where the car went
  std::vector<Vector3> car_states;
  std::vector<Vector3> measured_car_states;
  car_states.push_back(car.getActualState());
  measured_car_states.push_back(car.getMeasuredState());

  // let the car observe its environment
  Roomscanner scanner(car.lidar, rooms);
  
  FullLidarMeasurement full_measurement;
  scanner.scanRooms(car.getActualState(), 0., full_measurement);
  std::vector<FullLidarMeasurement> fms;
  fms.push_back(full_measurement);

  // now let's move the car
  for (int i=0; i<50; i++) {
    car.applyControlInput(Vector2(.2, 0.3), 0.1);
    if (i % 10 == 0) {
      car_states.push_back(car.getActualState());
      measured_car_states.push_back(car.getMeasuredState());
      scanner.scanRooms(car.getActualState(), 0., full_measurement);
      fms.push_back(full_measurement);
    } 
  }
  for (int i=0; i<50; i++) {
    car.applyControlInput(Vector2(.2, -0.3), 0.1);
    if (i % 10 == 0) {
      car_states.push_back(car.getActualState());
      measured_car_states.push_back(car.getMeasuredState());
      scanner.scanRooms(car.getActualState(), 0., full_measurement);
      fms.push_back(full_measurement);
    } 
  }
  // scanner.scanRooms(car.getActualState(), 0., full_measurement);
  // fms.push_back(full_measurement);
  
  std::thread writeToFile(writingToFileFunction, fms, car_states,
      measured_car_states, car.CAR_DIMENSIONS);
  writeToFile.join();

  plotting_thread.join();
}