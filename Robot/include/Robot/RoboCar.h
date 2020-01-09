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
  AccelerationSensor() {}

  /**
   * \brief given the cars velocity vectors returns a sensor measurement for
   * the cars angular velocity
   * \param [in] the 
   *    and angular velocity of the car
   * \param [out] the angular velocity measured by the sensor
   */
  double gyroscopeMeasurement(const Vector3& i_velocity) {
    // TODO implement noise
      return i_velocity(2);
  }

  // TODO implement noise
  // TODO figure out angular accelerations
  Vector2 linearAccelerationsMeasurement(const Vector3& i_velocity,
                                         const Vector3& i_velocity_old,
                                         const Vector3& state,
                                         const double wheel_base,
                                         const double gamma,
                                         const double dt) {
    Vector2 linear_part = (i_velocity.topRows(2) - i_velocity_old.topRows(2)) / dt;
    
    // in case gamma = 0 we don't have angular acceleration and just return
    if (abs(gamma) < 1e-9) {
      return linear_part;
    } 
    // now let's compute the acceleration caused by the circular motion
    // if we set a frame in the center of the turning radius, we can simply compute
    double turning_radius = wheel_base / tan(gamma);
    // the transformation matrix taking a vector from the coordinate frame of the 
    // turning point to the coordinate frame of the car
    Matrix3 C_E_T;
    C_E_T << 1, 0, 0,
             0, 1, -turning_radius,
             0, 0, 1;
    Vector3 E_r(0., turning_radius, 0.); // vector to car frame from turning frame
    Vector3 E_r_w2 = E_r * pow(i_velocity(2), 2);
    E_r_w2(2) = 1.;

    // transformation matrix from car frame to inertial coordinate frame 
    Matrix3 I_C_T;
    I_C_T << cos(state(2)), -sin(state(2)), state(0),
             sin(state(2)), cos(state(2)), state(1),
             0, 0, 1;
    Vector3 I_r_w2 = I_C_T * C_E_T * E_r_w2;

    return linear_part + I_r_w2.topRows(2);     
  }

  private:
  double sigma;
};

class RoboCar {
  friend class AccelerationSensor;
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
   * \param signal [in] the input DoF, signal(0) = gas, signal(1) = steering
   * \param dt [in] how long the signal should be applied
   */
  void applyControlInput(Vector2 signal, double dt);

  void updateStateFromIMUMeasurement(const double dt);

  Vector2 getAccelerationSensorMeasurement(const double gamma, const double dt);

  const Vector2 getPosition();
  const Vector3& getActualState();
  const Vector3& getMeasuredState();

  Lidar lidar;
  // width and length of the cars
  const std::pair<double,double> CAR_DIMENSIONS = std::pair<double,double>(0.05, 0.25);

  private:
  Vector3 actual_state; // (state(0), state(1)) = (x,y) // state(2) = theta
  Vector3 measured_state; // (state(0), state(1)) = (x,y) // state(2) = theta
  
  Matrix3 transformation; // TODO figure out if this is needed or not
  Vector2 signal; // input signal at current timestep
  Vector3 i_velocity; // velocity in inertial frame
  Vector3 i_velocity_old; // // velocity in inertial frame of previous time step
  Vector2 measured_velocity; 

  AccelerationSensor imu;

  const double MAX_FORWARD_VELOCITY = 2; // in m/s
  const double MAX_REVERSE_VELOCITY = 1; // in m/s
  const double MAX_STEERING_ANGLE = PI / 8; // in radians
  const double WHEEL_BASE = 0.2; // in m
};

#endif