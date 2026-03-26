#ifndef MODBUSRTUMASTER_H
#define MODBUSRTUMASTER_H

#include <iostream>
#include <thread>
#include <string>
#include <mutex>

//OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

// Simple POSIX serial wrapper (replaces ROS1 serial::Serial dependency)
class PosixSerial
{
public:
  PosixSerial();
  ~PosixSerial();

  void setPort(const std::string &port);
  void setBaudrate(uint32_t baudrate);
  void open();
  void close();
  bool isOpen() const;
  size_t write(const uint8_t *data, size_t size);
  size_t read(uint8_t *buffer, size_t size);
  void flush();

private:
  int fd_;
  std::string port_;
  uint32_t baudrate_;
  bool is_open_;
};

class ModbusRTUMaster
{
public:
  ModbusRTUMaster(const std::string portName, const uint32_t baudRate);
  ~ModbusRTUMaster();

  uint8_t SetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data);
  uint8_t GetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data);

protected:
  static uint16_t ModBusCRC(const uint8_t *data, const uint8_t length);

private:
  std::mutex mtx_;
  PosixSerial com_;
};

#endif // !MODBUSRTUMASTER_H
