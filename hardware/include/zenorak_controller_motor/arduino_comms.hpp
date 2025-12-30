#ifndef ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP
#define ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>


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

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  // std::string send_and_receive(const std::string &msg)
  // {
  //   serial_conn_.Write(msg);

  //   std::string response;
  //   serial_conn_.ReadLine(response, '\n', timeout_ms_);
  //   return response;
  // }
  void send_only(const std::string &msg)
  {
    serial_conn_.Write(msg);
  }


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


  // void send_empty_msg()
  // {
  //   std::string response = send_msg("\r");
  // }
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


  
  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    send_only(ss.str());
  }
  

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // ZENORAK_CONTROLLER_MOTOR_ARDUINO_COMMS_HPP