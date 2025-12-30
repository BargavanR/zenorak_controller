#ifndef ZENORAK_CONTROLLER_MOTOR_WHEEL_HPP
#define ZENORAK_CONTROLLER_MOTOR_WHEEL_HPP

// Simple helper class representing a wheel with an encoder.
// This file contains only a very small POD-like class used by the
// hardware interface to store the wheel's runtime state and perform
// small conversions (encoder counts -> radians).

#include <string>
#include <cmath>


class Wheel
{
    public:

    // public fields (kept simple intentionally):
    // name: the joint name (as used in the robot description / ros2_control)
    std::string name = "";
    // enc: last read encoder count (integer, as reported by microcontroller)
    int enc = 0;
    // cmd: commanded velocity in radians/sec (written by controller)
    double cmd = 0;
    // pos: last computed position in radians (from enc * rads_per_count)
    double pos = 0;
    // vel: last computed velocity in radians/sec
    double vel = 0;
    // rads_per_count: conversion factor from encoder counts to radians
    double rads_per_count = 0;

    // Default constructor - leaves everything at zero/empty.
    Wheel() = default;

    // Convenience constructor: set name and counts-per-revolution.
    // Example usage: Wheel("left_wheel_joint", 32);
    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
    }

    // setup: initialize the wheel's name and compute rads_per_count from
    // the counts-per-revolution value (encoders full scale).
    // Syntax: setup(<joint_name>, <counts_per_rev>)
    // Where it's used next: called from the hardware implementation during on_init().
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      // rads_per_count = 2*pi / counts_per_rev
      // This converts integer encoder ticks into radians of wheel rotation.
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    // calc_enc_angle: convert the stored encoder tick count to radians.
    // Syntax: double theta = calc_enc_angle();
    // Where it's called: in the hardware read() loop to update wheel.pos.
    // Output: position in radians (double).
    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }


};


#endif // ZENORAK_CONTROLLER_MOTOR_WHEEL_HPP
