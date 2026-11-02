#pragma once
#include <cstdlib>
#include <stdexcept>
#include <string>

// Only the upstream main's hard-coded SDK ports are parameterized.
inline unsigned z1_environment_port(const char* name, unsigned default_value) {
  const char* value = std::getenv(name);
  if (!value) return default_value;
  std::size_t consumed = 0;
  const std::string input(value);
  const auto result = std::stoul(input, &consumed);
  if (consumed != input.size() || result == 0 || result > 65535)
    throw std::invalid_argument(std::string("invalid UDP port: ") + name);
  return static_cast<unsigned>(result);
}
