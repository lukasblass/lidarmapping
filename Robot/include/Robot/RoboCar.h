#include <stdlib.h>
#include <types/types.h>
#include <Lidar/Lidar.h>

#ifndef ROBOCAR_H_
#define ROBOCAR_H_

class RoboCar {
  public:
  RoboCar(Lidar lidar, Vector3 initial_state);
  
  // TODO implement actual and measured accelerations and positions
  // to model a sensor on the car, e.g. by implementing a sensor class
  /**
   * \brief allows to specify what acceleration should be applied
   * over the time period dt
   */
  void applyAcceleration(const Vector2& acceleration, const double dt);
  const Vector2 getPosition();
  const Vector3& getState();

  Lidar lidar;
  
  private:
  Vector3 state; // (state(0), state(1)) = (x,y) // state(2) = theta
  Vector2 velocity;
};

#endif