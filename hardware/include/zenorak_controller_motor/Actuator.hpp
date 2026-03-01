#ifndef ZENORAK_CONTROLLER_MOTOR_ACTUATOR_HPP
#define ZENORAK_CONTROLLER_MOTOR_ACTUATOR_HPP

// Actuator helper class (kept for historical/compatibility reasons).
// If your robot does not have analog actuators (pots/ADCs), this file
// can be ignored. It models a simple actuator that maps potentiometer
// counts to an angular position and vice-versa.

#include <string>
#include <cmath>


// clamp: utility to constrain a value between min and max.
// Syntax: clamp(value, min, max)
// Returns: value if in range, otherwise nearest bound.
inline double clamp(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
};

#endif // ZENORAK_CONTROLLER_MOTOR_ACTUATOR_HPP
    public:

    // name: joint name in the robot description (if used)
    std::string name = "";
    // raw_count: raw ADC or encoder reading from microcontroller
    int raw_count = 0; // raw potentiometer/encoder reading from Arduino
    // cmd: desired position command (radians)
    double cmd = 0;
    // pos: current position (computed from raw_count)
    double pos = 0;
    // vel: computed velocity (pos change / dt)
    double vel = 0;
    // rads_per_count: conversion factor (if used)
    double rads_per_count = 0;
    // counts_per_rev: counts representing one revolution (if applicable)
    int counts_per_rev = 1; // divider to compute rads_per_count



    Actuator() = default;

    // convenience constructor: initializes name
    Actuator(const std::string &link_name)
    {
      setup(link_name);
    }

    // setup: set the name. In some robots this would also compute
    // rads_per_count from counts_per_rev; here it's a placeholder.
    void setup(const std::string &link_name)
    {
      name = link_name;

    }

    // calc_enc_angle: convert raw_count to radians using rads_per_count
    // Output: radians (double)
    double calc_enc_angle()
    {
      return raw_count * rads_per_count;
    }
    
    // angleToPot: convert a desired angle (degrees) into a potentiometer
    // integer value between pot_min and pot_max.
    // Syntax: angleToPot(theta_deg, logical_min_deg, logical_max_deg, pot_min, pot_max)
    // Used when sending actuator targets to microcontroller.
    int angleToPot(double theta,
                  double logical_min_deg, double logical_max_deg,
                  int pot_min, int pot_max)
    {


      theta = clamp(theta, logical_min_deg, logical_max_deg);

      double ratio =
        (theta - logical_min_deg) / (logical_max_deg - logical_min_deg);

      return static_cast<int>(pot_min + ratio * (pot_max - pot_min));
    }

    // potToAngle: convert raw potentiometer integer to an angle in degrees
    // Syntax: potToAngle(pot_value, logical_min_deg, logical_max_deg, pot_min, pot_max)
    // Output: angle in degrees as double (note: the original code labeled this
    // degrees->radians conversion but returned degrees; calling code should be aware.)
    double potToAngle(int pot_value,
                      double logical_min_deg, double logical_max_deg,
                      int pot_min, int pot_max)
    {
      // Clamp raw pot to safe range
      pot_value = static_cast<int>(
        clamp(static_cast<double>(pot_value),
              static_cast<double>(pot_min),
              static_cast<double>(pot_max)));

      // Normalize (0 → 1)
      double ratio =
        static_cast<double>(pot_value - pot_min) /
        static_cast<double>(pot_max - pot_min);

      // Scale to logical angle (degrees)
      double theta_deg =
        logical_min_deg + ratio * (logical_max_deg - logical_min_deg);

      // Note: original implementation returns degrees (not converted to radians).
      // If you expect radians, divide by 180 and multiply by M_PI here.
      return theta_deg ;
    }



};
};


#endif // ZENORAK_CONTROLLER_MOTOR_WHEEL_HPP
