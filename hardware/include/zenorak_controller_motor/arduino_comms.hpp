#ifndef ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP
#define ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP

// #include <cstring>
// ArduinoComms: small convenience class that wraps a serial connection
// to an Arduino (or other microcontroller) using LibSerial. It sends simple
// ASCII commands and reads ASCII responses. The hardware plugin uses this
// class to read wheel encoder values and to send motor commands.

#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


// convert_baud_rate: helper that maps an integer baud rate (e.g. 115200)
// to LibSerial::BaudRate enum used by the library. If an unsupported baud
// rate is passed, it logs a message and returns a safe default.
LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  // default ctor
  ArduinoComms() = default;

  // connect: open the serial device at the requested baud and store timeout
  // Syntax: connect("/dev/ttyACM0", 115200, 1000);
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  // disconnect: close the underlying serial connection
  void disconnect()
  {
    serial_conn_.Close();
  }

  // connected: query if the port is currently open
  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  // send_only: send a message and do not wait for a response
  // Example usage: send_only("m 10 -10\r") to set motor speeds
  void send_only(const std::string &msg)
  {
    serial_conn_.Write(msg);
  }


  // send_msg: send message and attempt to read a single line response.
  // Returns true on success (response filled), false on timeout
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
    // DO NOT throw
    // DO NOT print every time
    return false;  // timeout happened
  }
}


  // read_encoder_values: ask the microcontroller for encoder counts.
  // Expected microcontroller reply: two integers separated by space and newline,
  // e.g. "123 -456\n". On success this fills val_1 and val_2 and returns true.
  bool read_encoder_values(int &val_1, int &val_2)
  {
    std::string response;
    bool ok = send_msg("e\r", response);

    if (!ok) {
      // timeout → keep last values
      return false;
    }

    std::stringstream ss(response);
    ss >> val_1 >> val_2;
    return true;
  }


  // set_motor_values: build and send the ASCII command the Arduino expects
  // Format used: "m <left_counts> <right_counts>\r"
  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_only(ss.str());
  }


private:
    // serial connection object from LibSerial
    LibSerial::SerialPort serial_conn_;
    // read timeout in milliseconds used by ReadLine
    int timeout_ms_;
};

#endif // ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP