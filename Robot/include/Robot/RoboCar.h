#include <stdlib.h>
#include <random>
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
  AccelerationSensor() {
    distribution_gyro = std::normal_distribution<double>(0., 0.1);
    distribution_acc = std::normal_distribution<double>(0., 0.05);
    add_noise = true;
  }
  
  AccelerationSensor(const double sigma_gyro, const double sigma_acc) {
    distribution_gyro = std::normal_distribution<double>(0., sigma_gyro);
    distribution_acc = std::normal_distribution<double>(0., sigma_acc);
  }

  /**
   * \brief given the cars velocity vectors returns a sensor measurement for
   * the cars angular velocity
   * \param [in] the 
   *    and angular velocity of the car
   * \param [out] the angular velocity measured by the sensor
   */
  double gyroscopeMeasurement(const Vector3& i_velocity) {
    // TODO implement noise
    if (add_noise) {
      return i_velocity(2) + distribution_gyro(generator);
    } else {
      return i_velocity(2);
    }
  }

  Vector2 linearAccelerationsMeasurement(const Vector3& i_velocity,
                                         const Vector3& i_velocity_old,
                                         const Vector3& state,
                                         const double dt) {
    Vector2 linear_part = (i_velocity.topRows(2) - i_velocity_old.topRows(2)) / dt;
    Vector2 acceleration = linear_part;

    double angular_acceleration = (i_velocity(2) - i_velocity_old(2)) / dt;
    Vector3 psi(0., 0., angular_acceleration);
    Vector3 omega(0., 0., i_velocity(2));

    // transformation matrix from car frame to inertial coordinate frame 
    Matrix3 I_C_T;
    I_C_T << cos(state(2)), -sin(state(2)), state(0),
              sin(state(2)), cos(state(2)), state(1),
              0, 0, 1;
    
    // the cars origin in inertial coordinate frame
    Vector3 I_car_origin = I_C_T * Vector3(0., 0., 0.);

    Vector3 angular_acc_part = psi.cross(I_car_origin);
    Vector3 angular_vel_part = omega.cross(omega.cross(I_car_origin));

    acceleration = acceleration + angular_acc_part.topRows(2) + 
                    angular_vel_part.topRows(2);
    if (add_noise) {
      return acceleration + Vector2(distribution_acc(generator), distribution_acc(generator));
    } else {
      return acceleration;
    } 
  }

  private:
  std::default_random_engine generator;
  std::normal_distribution<double> distribution_gyro;
  std::normal_distribution<double> distribution_acc;
  bool add_noise;
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
   * \param signal [in] the input DoF, signal(0) = gas, signal(1) = steering
   * \param dt [in] how long the signal should be applied
   */
  void applyControlInput(Vector2 signal, double dt);

  void updateStateFromIMUMeasurement(const double dt);

  const Vector2 getPosition();
  const Vector3& getActualState();
  const Vector3& getMeasuredState();

  Lidar lidar;
  // width and length of the cars
  const std::pair<double,double> CAR_DIMENSIONS = std::pair<double,double>(0.05, 0.25);

  private:
  Vector2 signal; // input signal at current timestep

  Vector3 actual_state; // (state(0), state(1)) = (x,y) // state(2) = theta
  Vector3 actual_state_old; // (state(0), state(1)) = (x,y) // state(2) = theta
  Vector3 measured_state; // (state(0), state(1)) = (x,y) // state(2) = theta

  Vector3 i_velocity; // velocity in inertial frame
  Vector3 i_velocity_old; // velocity in inertial frame of previous time step
  Vector2 measured_velocity;

  AccelerationSensor imu;

  const double MAX_FORWARD_VELOCITY = 2; // in m/s
  const double MAX_REVERSE_VELOCITY = 1; // in m/s
  const double MAX_STEERING_ANGLE = PI / 8; // in radians
  const double WHEEL_BASE = 0.2; // in m
};

#endif