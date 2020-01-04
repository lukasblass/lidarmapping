#include <Robot/RoboCar.h>
#include <iostream>
#include <math.h>

RoboCar::RoboCar(Lidar lidar_, Vector3 initial_state_) :
  lidar(lidar_), state(initial_state_), i_velocity(Vector3(0., 0., 0.)) {
}

double RoboCar::mapGasPedalToVelocity(double input_gas) {
  if (abs(input_gas) > 1) {
    return 100; // TODO figure out what to do here
  }
  if (input_gas > 0) {
    return input_gas * MAX_FORWARD_VELOCITY;
  } else {
    return input_gas * MAX_REVERSE_VELOCITY;
  }
}

double RoboCar::mapSteeringWheelToAngularVelocity(const double input_steering, 
                const double linear_velocity) {
  if (abs(input_steering) > 1) {
    return 100; // figure out what to do here
  }
  double angular_vel = (linear_velocity / WHEEL_BASE) * 
                        tan(input_steering * MAX_STEERING_ANGLE);
  return angular_vel;
}

// we are given control signal that contains information
// about the speed and the steering of the car
// this function applies these control signals over the 
// time period dt. I.e. it computes the linear and angular
// velocity of the car as well as the new position
void RoboCar::applyControlInput(Vector2 signal, double dt) {
  // first make sure the input is valid
  if (abs(signal(0) > 1. || abs(signal(1)) > 1.)) {
    std::cout << "Car Control inputs are invalid" << std::endl;
    return;
  }
  // now from the signal determine the cars velocities
  double c_v = mapGasPedalToVelocity(signal(0)); // cars velocity along its x-axis
  i_velocity(0) = c_v * cos(state(2));
  i_velocity(1) = c_v * sin(state(2));
  i_velocity(2) = mapSteeringWheelToAngularVelocity(signal(1), c_v);
  
  // now we can use the information about the velocity to 
  // determine the state of the car after dt
  state = state + i_velocity * dt;
  // make sure the orientation stays within [0, 2*PI]
  if (state(2) > 2 * PI) {
    state(2) -= 2 * PI;
  } else if (state(2) < 0) {
    state(2) += 2 * PI;
  }
}

const Vector2 RoboCar::getPosition() {
  return Vector2(state(0), state(1));;
}

const Vector3& RoboCar::getState() {
  return state;
}