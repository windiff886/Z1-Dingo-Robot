#undef NDEBUG
#include "modbus_rtu_master.h"
#include <cassert>

int main() {
  bool failed = false;
  try {
    ModbusRTUMaster master("unavailable", 115200);
  } catch (const serial::IOException &) {
    failed = true;
  }
  assert(failed);

  ModbusRTUMaster master("in-memory", 115200);
  uint16_t registers[3] = {10, 0, 0};
  // A successful open does not make a missing device reply successful.
  assert(master.SetMultipleRegisters(1, 6, 3, registers) == 0);
  assert(master.GetMultipleRegisters(1, 0, 3, registers) == 0);
}
