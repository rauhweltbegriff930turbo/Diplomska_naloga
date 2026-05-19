// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include "rmi/rmi.hpp"

namespace
{
template <typename Fn>
bool RunStep(const std::string& name, Fn&& fn)
{
  std::cout << "[STEP] " << name << std::endl;
  try
  {
    fn();
    std::cout << "[ OK ] " << name << std::endl;
    return true;
  }
  catch (const std::exception& e)
  {
    std::cout << "[FAIL] " << name << ": " << e.what() << std::endl;
    return false;
  }
}
}  // namespace

int main(int argc, char* argv[])
{
  if (argc < 2 || argc > 4)
  {
    std::cerr << "Usage: " << argv[0] << " <robot_ip> [rmi_port] [group_mask]" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string robot_ip = argv[1];
  const uint16_t rmi_port = argc == 3 ? static_cast<uint16_t>(std::stoi(argv[2])) : 16001;
  const std::optional<uint8_t> group_mask =
      argc == 4 ? std::optional<uint8_t>(static_cast<uint8_t>(std::stoi(argv[3]))) : std::nullopt;

  std::cout << "Connecting to FANUC RMI at " << robot_ip << ":" << rmi_port;
  if (group_mask.has_value())
  {
    std::cout << " with group mask " << static_cast<int>(*group_mask);
  }
  std::cout << std::endl;
  rmi::RMIConnection connection(robot_ip, rmi_port, group_mask);

  bool ok = true;
  ok &= RunStep("connect", [&] { connection.connect(std::nullopt); });

  bool first_path_ok = true;
  first_path_ok &= RunStep("getStatus", [&] { connection.getStatus(std::nullopt); });
  first_path_ok &= RunStep("reset", [&] { connection.reset(std::nullopt); });
  first_path_ok &= RunStep("initializeRemoteMotion", [&] { connection.initializeRemoteMotion(std::nullopt); });

  if (!first_path_ok)
  {
    std::cout << "Primary initialization path failed; trying driver recovery path." << std::endl;
    ok = false;
    RunStep("abort", [&] { connection.abort(std::nullopt); });
    RunStep("reset after abort", [&] { connection.reset(std::nullopt); });
    RunStep("getStatus after abort", [&] { connection.getStatus(std::nullopt); });
    RunStep("initializeRemoteMotion after abort", [&] { connection.initializeRemoteMotion(std::nullopt); });
  }

  RunStep("disconnect", [&] { connection.disconnect(std::nullopt); });

  if (!first_path_ok)
  {
    std::cout << "RMI smoke test failed." << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "RMI smoke test completed successfully." << std::endl;
  return EXIT_SUCCESS;
}
