#include <stdlib.h>
#include <types/types.h>
#include <Lidar/Lidar.h>

#ifndef ROBOCAR_H_
#define ROBOCAR_H_

/**
 * \brief Models an IMU that's mounted on the car. The sensor has some
 * amount of noise associated with it such that the measured accelerations
 * are only so accurate
 * */
class AccelerationSensor {
  public:
  AccelerationSensor();

  /**
   * \brief given a true acceleration computes the sensor output
   * \param dof_acceleration [in] the linear and angular acceleration applied to the car
   * \param [out] the acceleration in x and y direction w.r.t to the sensors local coordinate frame
   */
  Vector2 gyroscopeMeasurement(const Vector2& dof_acceleration) {
      
  }

  Vector2 linearAccelerationsMeasurement(const Vector2& true_acceleration) {

  }

  private:
  double sigma;
};

class RoboCar {
  public:
  RoboCar(Lidar lidar, Vector3 initial_state);
  
  // TODO implement actual and measured accelerations and positions
  // to model a sensor on the car, e.g. by implementing a sensor class
  /**
   * \brief takes gas pedal value and maps it to the corresponding velocity
   * of the car
   * \param input_gas [in] the gas pedal value in range [-1,1]
   * \param [out] the corresponding speed of the car
   */
  double mapGasPedalToVelocity(double input_gas);
  
  /**
   * \brief takes steering wheel value and maps it to the corresponding angular
   * velocity of the car. Notice the dependency on the linear velocity of the car
   * \param input_stering [in] the steering wheel value in range [-1,1]
   * \param linear_velocity [in] the velocity of the car along it's x-axis
   * \param [out] the corresponding angular velocity of the car
   */
  double mapSteeringWheelToAngularVelocity(const double input_steering,
                                           const double linear_velocity);

  /**
   * \brief applies the given gas pedal and steering wheel values to the car
   * for a period of dt and updates all dependent values (e.g. state)
   * \param signal [in] the input DoF
   * \param dt [in] how long the signal should be applied
   */
  void applyControlInput(Vector2 signal, double dt);

  const Vector2 getPosition();
  const Vector3& getState();

  Lidar lidar;
  // width and length of the cars
  const std::pair<double,double> CAR_DIMENSIONS = std::pair<double,double>(0.05, 0.25);

  private:
  Vector3 state; // (state(0), state(1)) = (x,y) // state(2) = theta
  
  Matrix3 transformation; // TODO figure out if this is needed or not
  Vector3 i_velocity; // velocity in inertial frame

  const double MAX_FORWARD_VELOCITY = 2; // in m/s
  const double MAX_REVERSE_VELOCITY = 1; // in m/s
  const double MAX_STEERING_ANGLE = PI / 8; // in radians
  const double WHEEL_BASE = 0.2; // in m
};

#endif