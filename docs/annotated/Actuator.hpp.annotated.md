# Annotated: Actuator.hpp

This document annotates the `Actuator` helper class. It explains every logical line/block and shows where each method is used.

---

```cpp
#ifndef ZENORAK_CONTROLLER_ACTUATOR_ACTUATOR_HPP
#define ZENORAK_CONTROLLER_ACTUATOR_ACTUATOR_HPP
```

- Include guard to prevent duplicate inclusion.

```cpp
#include <string>
#include <cmath>
```

- Includes for std::string and math helpers.

```cpp
inline double clamp(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}
```

- Small utility to clamp a floating value between limits. Used by `angleToPot` and `potToAngle`.

```cpp
class Actuator
{
    public:

    std::string name = "";
    int raw_count = 0; // raw potentiometer/encoder reading from Arduino
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;
    int counts_per_rev = 1; // divider to compute rads_per_count
```

- Data members:
  - `name`: joint name used when declaring interfaces.
  - `raw_count`: last raw ADC/encoder reading from microcontroller.
  - `cmd`: command value written by controllers (position in this package).
  - `pos` and `vel`: current state values exported to ros2_control.
  - `rads_per_count` and `counts_per_rev`: conversion constants for encoders (if used).

```cpp
    Actuator() = default;

    Actuator(const std::string &link_name)
    {
      setup(link_name);
    }

    void setup(const std::string &link_name)
    {
      name = link_name;

    }
```

- Constructors and `setup` method: `setup` assigns a friendly joint name and could be extended to compute conversion factors.

```cpp
    double calc_enc_angle()
    {
      return raw_count * rads_per_count;
    }
```

- Convert a raw encoder count to an angle in radians using `rads_per_count`.

```cpp
    int angleToPot(double theta,
                  double logical_min_deg, double logical_max_deg,
                  int pot_min, int pot_max)
    {

      theta = clamp(theta, logical_min_deg, logical_max_deg);

      double ratio =
        (theta - logical_min_deg) / (logical_max_deg - logical_min_deg);

      return static_cast<int>(pot_min + ratio * (pot_max - pot_min));
    }
```

- Map a logical angle in degrees to a hardware potentiometer range (integer counts).
- Usage: called by `diffbot_system.cpp::write()` to convert `Actuator.cmd` into `a1_target` etc.

```cpp
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

      // Degrees → radians
      return theta_deg ;
    }
```

- Convert a potentiometer count (ADC) into a logical angle in degrees.
- Note: this function returns degrees as written; if radians are required, multiply by `M_PI/180.0` externally or update here.

```cpp
#endif // ZENORAK_CONTROLLER_ACTUATOR_WHEEL_HPP
```

Cross references:

- `angleToPot` used in `hardware/diffbot_system.cpp::write()` to compute target counts.
- `potToAngle` used in `hardware/diffbot_system.cpp::read()` to convert raw ADC readings to logical joint positions.

Production summary:

- `Actuator` is a tiny data-and-conversion helper. It keeps the last raw reading and computes friendly positions and velocities that the hardware interface exports to ROS controllers. It's intentionally simple so you can adapt the mapping parameters to your specific potentiometer or encoder.
