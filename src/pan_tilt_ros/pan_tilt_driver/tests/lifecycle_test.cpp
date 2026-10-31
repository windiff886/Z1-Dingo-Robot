#undef NDEBUG
#include "PanTiltDriver.h"

#include <atomic>
#include <cassert>
#include <chrono>
#include <thread>

namespace {
IQR::PanTiltDriver *driver;
std::atomic<unsigned> reads{0};
unsigned destroyed = 0;
}

ModbusRTUMaster::ModbusRTUMaster(const std::string, const uint32_t) {}
ModbusRTUMaster::~ModbusRTUMaster() {
  assert(driver->getThreadStatus() == QThread::THREAD_STATUS_EXIT);
  ++destroyed;
}
uint8_t ModbusRTUMaster::SetMultipleRegisters(
    const uint8_t id, const uint16_t address, const uint16_t length,
    uint16_t *data) {
  assert(id == 1 && address == 6 && length == 3);
  assert(data[0] == 10 && data[1] == 0 && data[2] == 0);
  return 1;
}
uint8_t ModbusRTUMaster::GetMultipleRegisters(
    const uint8_t id, const uint16_t address, const uint16_t length,
    uint16_t *data) {
  assert(id == 1 && address == 0 && length == 20);
  ++reads;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (unsigned i = 0; i < length; ++i)
    data[i] = 0;
  data[0] = id;
  return 1;
}

int main() {
  // Shutdown during an active read must join before destroying the serial owner.
  for (unsigned i = 0; i < 3; ++i) {
    const auto previous = reads.load();
    driver = new IQR::PanTiltDriver(1, "in-memory");
    while (reads.load() == previous)
      std::this_thread::yield();
    delete driver;
  }
  assert(destroyed == 3);
}
