# Annotated: arduino_comms.hpp

This document contains the original `arduino_comms.hpp` contents with friendly, detailed explanations for each section/line, plus cross-references to where functions are used.

---

```cpp
#ifndef ZENORAK_CONTROLLER_ACTUATOR_ARDUINO_COMMS_HPP
#define ZENORAK_CONTROLLER_ACTUATOR_ARDUINO_COMMS_HPP
```

- Standard C++ include guard to prevent double inclusion at compile time.

```cpp
#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
```

- Includes for string streams (to build messages), LibSerial for serial comms, and iostream for debug prints.

```cpp
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    ...
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}
```

- What it is: small helper that maps an integer baud rate (e.g., 115200) into LibSerial's enum type.
- Where used: called in `connect()` to set serial speed.

```cpp
class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }
```

- What it is: `connect` opens the serial device path (e.g., `/dev/ttyACM0`) and sets the baud rate. It stores the timeout for future reads.
- Syntax: `serial_conn_.Open(...)` uses LibSerial API.
- Where called: `on_configure()` in `diffbot_system.cpp`.

```cpp
  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }
```

- Simple wrappers to close the serial port and check if it's open.

```cpp
  void send_only(const std::string &msg)
  {
    serial_conn_.Write(msg);
  }
```

- Send bytes to Arduino without expecting a reply.

```cpp
  bool send_msg(const std::string &msg_to_send, std::string &response)
{
  try
  {
    serial_conn_.Write(msg_to_send);
    serial_conn_.ReadLine(response, '\n', timeout_ms_);
    return true;   // success
  }
  catch (const LibSerial::ReadTimeout&)
  {
    return false;  // timeout happened
  }
}
```

- What it is: send a message then block until a newline-terminated response or timeout. Returns `true` if response received.
- Where used: `read_actuator_values` uses this to request `a\r` and parse returned counts.

```cpp
  bool read_actuator_values(int &val_1, int &val_2, int &val_3)
  {
    std::string response;
    bool ok = send_msg("a\r", response);

    if (!ok) {
      return false;
    }

    std::stringstream ss(response);
    ss >> val_1 >> val_2 >> val_3;
    return true;
  }
```

- What it is: request three actuator ADC counts from the Arduino. The Arduino is expected to reply with a space-separated line like `512 600 450\n`.
- Output: sets `val_1..3` to parsed integers; returns true when successful.
- Where it's used: called by `Zenorak_Hardware_Actuator::read()` to update actuator readings.

```cpp
  void set_actuator_positions(int val_1, int val_2, int val_3)
  {
    std::stringstream ss;
    ss << "s " << val_1 << " " << val_2 << " " << val_3 << "\r";
    send_only(ss.str());
  }
```

- What it is: send an `s v1 v2 v3\r` message to the Arduino to command actuator positions in hardware counts.
- Where it's used: `Zenorak_Hardware_Actuator::write()` constructs targets and calls this method.

```cpp
  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    // send_msg(ss.str());
  }
```

- This prepares a PID set-string. The actual send is commented out; you may enable `send_msg` if the microcontroller supports and responds to this command.

```cpp
private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // ZENORAK_CONTROLLER_ACTUATOR_ARDUINO_COMMS_HPP
```

- Private members hold the serial port object and configured read timeout (ms).

Cross references summary:

- `read_actuator_values` is used in `hardware/diffbot_system.cpp::read()` to obtain current measurement values.
- `set_actuator_positions` is used in `hardware/diffbot_system.cpp::write()` to send actuator commands.

Production notes:

- This class is a small, synchronous protocol layer to an Arduino. In production, consider making reads non-blocking or using a dedicated I/O thread to avoid blocking the real-time control loop.
- Validate Arduino replies to detect malformed lines.
