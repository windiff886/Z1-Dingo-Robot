#include "modbus_rtu_master.h"

#include <cstdio>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

#define REBACK_SLEEP_MS 5

// ---------- PosixSerial implementation ----------

PosixSerial::PosixSerial() : fd_(-1), baudrate_(115200), is_open_(false) {}

PosixSerial::~PosixSerial()
{
  if (is_open_)
    close();
}

void PosixSerial::setPort(const std::string &port) { port_ = port; }
void PosixSerial::setBaudrate(uint32_t baudrate) { baudrate_ = baudrate; }

void PosixSerial::open()
{
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0)
  {
    std::cerr << "Unable to open serial port: " << port_ << " - " << strerror(errno) << std::endl;
    return;
  }

  // Configure port
  struct termios tty;
  memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd_, &tty) != 0)
  {
    std::cerr << "tcgetattr failed: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return;
  }

  // Set baud rate
  speed_t baud;
  switch (baudrate_)
  {
  case 9600:
    baud = B9600;
    break;
  case 19200:
    baud = B19200;
    break;
  case 38400:
    baud = B38400;
    break;
  case 57600:
    baud = B57600;
    break;
  case 115200:
    baud = B115200;
    break;
  default:
    baud = B115200;
    break;
  }
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS; // No flow control
  tty.c_cflag |= CLOCAL | CREAD;

  // Raw mode
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~OPOST;
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Timeout: 50ms (5 deciseconds)
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1; // 100ms timeout per read

  if (tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    std::cerr << "tcsetattr failed: " << strerror(errno) << std::endl;
    ::close(fd_);
    fd_ = -1;
    return;
  }

  tcflush(fd_, TCIOFLUSH);
  fcntl(fd_, F_SETFL, 0); // Blocking mode

  is_open_ = true;
}

void PosixSerial::close()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  is_open_ = false;
}

bool PosixSerial::isOpen() const { return is_open_; }

size_t PosixSerial::write(const uint8_t *data, size_t size)
{
  if (!is_open_)
    return 0;
  ssize_t n = ::write(fd_, data, size);
  return (n > 0) ? static_cast<size_t>(n) : 0;
}

size_t PosixSerial::read(uint8_t *buffer, size_t size)
{
  if (!is_open_)
    return 0;

  size_t total = 0;
  int retries = 10;
  while (total < size && retries > 0)
  {
    ssize_t n = ::read(fd_, buffer + total, size - total);
    if (n > 0)
    {
      total += static_cast<size_t>(n);
    }
    else
    {
      retries--;
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  return total;
}

void PosixSerial::flush()
{
  if (is_open_)
    tcflush(fd_, TCIOFLUSH);
}

// ---------- ModbusRTUMaster implementation ----------

ModbusRTUMaster::ModbusRTUMaster(const std::string portName, const uint32_t baudRate)
{
  com_.setPort(portName);
  com_.setBaudrate(baudRate);
  com_.open();

  if (com_.isOpen())
  {
    std::cout << "open serial port:" << portName << " successful!!" << std::endl;
  }
  else
  {
    std::cerr << "Unable to open serial port:" << portName << std::endl;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

ModbusRTUMaster::~ModbusRTUMaster()
{
  com_.close();
}

uint8_t ModbusRTUMaster::SetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data)
{
  if (!com_.isOpen())
    return 0;

  std::lock_guard<std::mutex> lck(mtx_);
  com_.flush();

  const uint8_t function_num = 0x10;
  uint8_t send_buffer[256] = {0};

  send_buffer[0] = slave_id;
  send_buffer[1] = function_num;
  send_buffer[2] = uint8_t((address & 0xff00) >> 8);
  send_buffer[3] = uint8_t((address & 0x00ff));
  send_buffer[4] = uint8_t((length & 0xff00) >> 8);
  send_buffer[5] = uint8_t((length & 0x00ff));
  send_buffer[6] = uint8_t((length & 0x00ff) * 2);
  uint8_t buffer_index = 6;
  for (uint16_t i = 0; i < length; i++)
  {
    send_buffer[7 + i * 2] = uint8_t((data[i] & 0xff00) >> 8);
    send_buffer[8 + i * 2] = uint8_t((data[i] & 0x00ff));
    buffer_index += 2;
  }

  uint16_t crc = ModBusCRC(send_buffer, buffer_index + 1);
  send_buffer[buffer_index + 1] = uint8_t((crc & 0x00ff));
  send_buffer[buffer_index + 2] = uint8_t((crc & 0xff00) >> 8);

  size_t buffer_length = buffer_index + 3;
  size_t write_length = com_.write(send_buffer, buffer_length);
  if (buffer_length != write_length)
  {
    com_.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t receive_buffer[256] = {0};
  size_t receive_length = 8;
  size_t read_length = com_.read(receive_buffer, receive_length);

  if (receive_length != read_length)
  {
    com_.flush();
    return 0;
  }
  if (receive_buffer[0] != slave_id || receive_buffer[1] != function_num)
  {
    com_.flush();
    return 0;
  }

  uint16_t temp_address = uint16_t((receive_buffer[2] << 8) | receive_buffer[3]);
  uint16_t temp_length = uint16_t((receive_buffer[4] << 8) | receive_buffer[5]);
  if (temp_address != address || temp_length != length)
  {
    com_.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(receive_buffer, receive_length - 2);
  uint16_t rec_crc_ = uint16_t((receive_buffer[receive_length - 1] << 8) | (receive_buffer[receive_length - 2]));
  if (rec_crc != rec_crc_)
  {
    com_.flush();
    return 0;
  }

  com_.flush();
  return 1;
}

uint8_t ModbusRTUMaster::GetMultipleRegisters(const uint8_t slave_id, const uint16_t address, const uint16_t length, uint16_t *data)
{
  if (!com_.isOpen())
    return 0;

  std::lock_guard<std::mutex> lck(mtx_);
  com_.flush();

  const uint8_t function_num = 0x03;
  uint8_t send_buffer[8] = {0};

  send_buffer[0] = slave_id;
  send_buffer[1] = function_num;
  send_buffer[2] = uint8_t((address & 0xff00) >> 8);
  send_buffer[3] = uint8_t((address & 0x00ff));
  send_buffer[4] = uint8_t((length & 0xff00) >> 8);
  send_buffer[5] = uint8_t((length & 0x00ff));
  uint16_t crc = ModBusCRC(send_buffer, 6);
  send_buffer[6] = uint8_t((crc & 0x00ff));
  send_buffer[7] = uint8_t((crc & 0xff00) >> 8);

  size_t write_length = com_.write(send_buffer, 8);
  if (write_length != 8)
  {
    com_.flush();
    return 0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(REBACK_SLEEP_MS));

  uint8_t receive_buffer[256] = {0};
  size_t receive_length = (length * 2) + 5;
  size_t read_length = com_.read(receive_buffer, receive_length);

  if (receive_length != read_length || receive_buffer[0] != slave_id ||
      receive_buffer[1] != function_num || receive_buffer[2] != (length * 2))
  {
    com_.flush();
    return 0;
  }

  uint16_t rec_crc = ModBusCRC(receive_buffer, receive_length - 2);
  uint16_t rec_crc_ = uint16_t((receive_buffer[receive_length - 1] << 8) | (receive_buffer[receive_length - 2]));
  if (rec_crc != rec_crc_)
  {
    com_.flush();
    return 0;
  }

  for (uint16_t i = 0; i < length; i++)
  {
    data[i] = uint16_t((receive_buffer[3 + i * 2] << 8) | (receive_buffer[4 + i * 2]));
  }

  com_.flush();
  return 1;
}

uint16_t ModbusRTUMaster::ModBusCRC(const uint8_t *data, const uint8_t length)
{
  uint8_t j;
  uint16_t crc;
  uint8_t len = length;
  crc = 0xFFFF;

  while (len--)
  {
    crc = crc ^ *data++;
    for (j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
      {
        crc = (crc >> 1) ^ 0xA001;
      }
      else
      {
        crc = crc >> 1;
      }
    }
  }

  return crc;
}
