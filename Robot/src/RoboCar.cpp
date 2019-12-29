#include <Robot/RoboCar.h>
#include <iostream>
#include <math.h>

RoboCar::RoboCar(Lidar lidar_, Vector3 initial_state_) :
  lidar(lidar_), state(initial_state_), velocity(Vector2(0., 0.)) {
}

void RoboCar::applyAcceleration(const Vector2& acceleration, const double dt) {
  velocity += dt * acceleration;
  
  state(0) = state(0) + dt * velocity(0) * cos(state(2));
  state(1) = state(1) + dt * velocity(0) * sin(state(2));
  state(2) += dt * velocity(1);
}

const Vector2 RoboCar::getPosition() {
  return Vector2(state(0), state(1));;
}

const Vector3& RoboCar::getState() {
  return state;
}