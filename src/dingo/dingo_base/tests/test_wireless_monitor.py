#!/usr/bin/env python3
"""Compile the wireless callback against in-memory getifaddrs results."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class WirelessMonitorTests(unittest.TestCase):
    def test_os_failure_empty_address_and_ip_detection(self):
        source = (Path(__file__).resolve().parents[1]
                  / 'src/dingo_diagnostic_updater.cpp').read_text()
        start = source.index('void DingoDiagnosticUpdater::wirelessMonitorCallback')
        callback = source[start:source.index('\n}\n', start) + 2]
        fixture = r'''
#undef NDEBUG
#include <cassert>
#include <cerrno>
#include <cstring>
#include <ifaddrs.h>
#include <sstream>
#include <string>
#include <sys/socket.h>

std::string warning;
#define ROS_WARN_STREAM(message) do { std::ostringstream text; text << message; warning = text.str(); } while (false)
namespace ros { struct TimerEvent {}; }
namespace std_msgs { struct Bool { bool data; }; }
namespace dingo_base {
struct Publisher {
  bool value = true;
  unsigned calls = 0;
  void publish(const std_msgs::Bool &message) { value = message.data; ++calls; }
};
class DingoDiagnosticUpdater {
public:
  std::string wireless_interface_ = "wlan0";
  Publisher wifi_connected_pub_;
  void wirelessMonitorCallback(const ros::TimerEvent &);
};
CALLBACK
}

bool fail = false;
unsigned freed = 0;
ifaddrs interfaces[3]{};
sockaddr addresses[2]{};
extern "C" int getifaddrs(ifaddrs **head) noexcept {
  if (fail) { errno = EIO; return -1; }
  *head = interfaces;
  return 0;
}
extern "C" void freeifaddrs(ifaddrs *head) noexcept {
  assert(head == interfaces);
  ++freed;
}

int main() {
  dingo_base::DingoDiagnosticUpdater node;
  fail = true;
  node.wirelessMonitorCallback({});
  assert(!node.wifi_connected_pub_.value && node.wifi_connected_pub_.calls == 1);
  assert(freed == 0 && warning.find("wlan0") != std::string::npos);

  fail = false;
  interfaces[0].ifa_name = const_cast<char *>("wlan0");
  interfaces[0].ifa_next = &interfaces[1];  // Matching interface, no address.
  interfaces[1].ifa_name = const_cast<char *>("eth0");
  interfaces[1].ifa_addr = &addresses[0];
  interfaces[1].ifa_next = &interfaces[2];
  addresses[0].sa_family = AF_INET;
  interfaces[2].ifa_name = const_cast<char *>("wlan0");
  interfaces[2].ifa_addr = &addresses[1];
  addresses[1].sa_family = AF_INET6;
  node.wirelessMonitorCallback({});
  assert(node.wifi_connected_pub_.value && freed == 1);

  // An existing interface without an IP address is disconnected.
  addresses[1].sa_family = AF_PACKET;
  node.wirelessMonitorCallback({});
  assert(!node.wifi_connected_pub_.value && freed == 2);
  assert(node.wifi_connected_pub_.calls == 3);
}
'''.replace('CALLBACK', callback)
        with tempfile.TemporaryDirectory(prefix='dingo-wireless-test-') as folder:
            cpp = Path(folder) / 'wireless.cpp'
            executable = Path(folder) / 'wireless'
            cpp.write_text(fixture)
            subprocess.run(['c++', '-std=c++11', '-Wall', '-Wextra',
                            str(cpp), '-o', str(executable)], check=True)
            subprocess.run([str(executable)], check=True)


if __name__ == '__main__':
    unittest.main()
