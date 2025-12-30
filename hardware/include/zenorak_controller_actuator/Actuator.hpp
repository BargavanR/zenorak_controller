#ifndef ZENORAK_CONTROLLER_ACTUATOR_ACTUATOR_HPP
#define ZENORAK_CONTROLLER_ACTUATOR_ACTUATOR_HPP

#include <string>
#include <cmath>


inline double clamp(double val, double min_val, double max_val)
{
  if (val < min_val) return min_val;
  if (val > max_val) return max_val;
  return val;
}

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



    Actuator() = default;

    Actuator(const std::string &link_name)
    {
      setup(link_name);
    }

    
    void setup(const std::string &link_name)
    {
      name = link_name;

    }

    double calc_enc_angle()
    {
      return raw_count * rads_per_count;
    }
    
    int angleToPot(double theta,
                  double logical_min_deg, double logical_max_deg,
                  int pot_min, int pot_max)
    {


      theta = clamp(theta, logical_min_deg, logical_max_deg);

      double ratio =
        (theta - logical_min_deg) / (logical_max_deg - logical_min_deg);

      return static_cast<int>(pot_min + ratio * (pot_max - pot_min));
    }
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




};


#endif // ZENORAK_CONTROLLER_ACTUATOR_WHEEL_HPP
