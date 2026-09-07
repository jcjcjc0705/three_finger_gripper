// Offline rescue tool. Talks to the serial port directly through DynamixelBus,
// with no ROS runtime involved -- use it when the ros2_control stack is stopped
// and therefore not holding the port. A motor stuck in Hardware Error, an
// unknown ID or a wrong baud rate are all cases the ROS services cannot reach.
//
//   ros2 run dynamixel_hardware dxl_offline --port /dev/ttyUSB0 scan
//   ros2 run dynamixel_hardware dxl_offline --port /dev/ttyUSB0 dump 11
//   ros2 run dynamixel_hardware dxl_offline --port /dev/ttyUSB0 reboot 13

#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "dynamixel_hardware/control_table.hpp"
#include "dynamixel_hardware/dynamixel_bus.hpp"

namespace ct = dynamixel_hardware::control_table;
using dynamixel_hardware::DynamixelBus;

namespace
{

void usage()
{
  std::cout <<
    "usage: dxl_offline [--port PATH] [--baud RATE] <command> [args]\n"
    "\n"
    "  scan [first] [last]              ping a range of IDs (default 1..30)\n"
    "  dump <id>                        print the full register table\n"
    "  read <id> <address> <size>       read one register\n"
    "  write <id> <address> <size> <v>  write one register\n"
    "  torque <id|all> <on|off>         enable/disable torque\n"
    "  led <id|all> <on|off>            indicator LED\n"
    "  reboot <id|all>                  clear Hardware Error Status\n"
    "  zero <id>                        write current position as the new zero\n"
    "\n"
    "  --port  default /dev/ttyUSB0\n"
    "  --baud  default 1000000 (X-series factory default is 57600)\n";
}

bool parse_onoff(const std::string & text, bool & out)
{
  if (text == "on" || text == "1" || text == "true") { out = true; return true; }
  if (text == "off" || text == "0" || text == "false") { out = false; return true; }
  return false;
}

std::vector<uint8_t> resolve_ids(DynamixelBus & bus, const std::string & text)
{
  std::vector<uint8_t> ids;
  if (text != "all")
  {
    ids.push_back(static_cast<uint8_t>(std::stoi(text)));
    return ids;
  }
  std::cout << "scanning for motors...\n";
  for (int id = 1; id <= 30; ++id)
  {
    uint16_t model = 0;
    if (bus.ping(static_cast<uint8_t>(id), model))
    {
      ids.push_back(static_cast<uint8_t>(id));
    }
  }
  return ids;
}

int cmd_scan(DynamixelBus & bus, int first, int last)
{
  int found = 0;
  for (int id = first; id <= last; ++id)
  {
    uint16_t model = 0;
    if (bus.ping(static_cast<uint8_t>(id), model))
    {
      int64_t err = 0;
      bus.read_register(static_cast<uint8_t>(id), ct::HARDWARE_ERROR_STATUS, 1, err);
      std::cout << "  id=" << std::setw(3) << id
                << "  model_number=" << std::setw(5) << model
                << "  hardware_error=" << err << '\n';
      ++found;
    }
  }
  std::cout << found << " motor(s) found on the bus\n";
  return found > 0 ? 0 : 1;
}

int cmd_dump(DynamixelBus & bus, uint8_t id)
{
  std::cout << std::left << std::setw(24) << "name" << std::right << std::setw(5) << "addr"
            << std::setw(5) << "size" << std::setw(12) << "value" << "  note\n";
  std::cout << std::string(70, '-') << '\n';
  for (const auto & reg : ct::READABLE_REGISTERS)
  {
    int64_t value = 0;
    const bool ok = bus.read_register(id, reg.address, reg.size, value);
    std::cout << std::left << std::setw(24) << reg.name << std::right << std::setw(5)
              << reg.address << std::setw(5) << static_cast<int>(reg.size) << std::setw(12);
    if (ok) { std::cout << value; } else { std::cout << "ERR"; }
    std::cout << "  " << reg.note << '\n';
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyUSB0";
  int baud = 1000000;

  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i)
  {
    const std::string a = argv[i];
    if (a == "--port" && i + 1 < argc) { port = argv[++i]; }
    else if (a == "--baud" && i + 1 < argc) { baud = std::stoi(argv[++i]); }
    else if (a == "-h" || a == "--help") { usage(); return 0; }
    else { args.push_back(a); }
  }

  if (args.empty()) { usage(); return 1; }

  DynamixelBus bus(port, baud);
  if (!bus.open())
  {
    std::cerr << bus.last_error() << '\n'
              << "(is the ros2_control stack still holding the port?)\n";
    return 1;
  }

  const std::string cmd = args[0];
  int rc = 0;

  try
  {
    if (cmd == "scan")
    {
      const int first = args.size() > 1 ? std::stoi(args[1]) : 1;
      const int last = args.size() > 2 ? std::stoi(args[2]) : 30;
      rc = cmd_scan(bus, first, last);
    }
    else if (cmd == "dump" && args.size() >= 2)
    {
      rc = cmd_dump(bus, static_cast<uint8_t>(std::stoi(args[1])));
    }
    else if (cmd == "read" && args.size() >= 4)
    {
      int64_t value = 0;
      if (!bus.read_register(
            static_cast<uint8_t>(std::stoi(args[1])),
            static_cast<uint16_t>(std::stoi(args[2])),
            static_cast<uint8_t>(std::stoi(args[3])), value))
      {
        std::cerr << bus.last_error() << '\n';
        rc = 1;
      }
      else { std::cout << value << '\n'; }
    }
    else if (cmd == "write" && args.size() >= 5)
    {
      if (!bus.write_register(
            static_cast<uint8_t>(std::stoi(args[1])),
            static_cast<uint16_t>(std::stoi(args[2])),
            static_cast<uint8_t>(std::stoi(args[3])), std::stoll(args[4])))
      {
        std::cerr << bus.last_error() << '\n';
        rc = 1;
      }
    }
    else if ((cmd == "torque" || cmd == "led") && args.size() >= 3)
    {
      bool on = false;
      if (!parse_onoff(args[2], on)) { usage(); rc = 1; }
      else
      {
        for (const auto id : resolve_ids(bus, args[1]))
        {
          const bool ok = (cmd == "torque") ? bus.set_torque(id, on) : bus.set_led(id, on);
          if (!ok) { std::cerr << bus.last_error() << '\n'; rc = 1; }
        }
      }
    }
    else if (cmd == "reboot" && args.size() >= 2)
    {
      for (const auto id : resolve_ids(bus, args[1]))
      {
        if (!bus.reboot(id)) { std::cerr << bus.last_error() << '\n'; rc = 1; }
        else { std::cout << "id=" << static_cast<int>(id) << " rebooted\n"; }
      }
    }
    else if (cmd == "zero" && args.size() >= 2)
    {
      if (!bus.set_zero(static_cast<uint8_t>(std::stoi(args[1]))))
      {
        std::cerr << bus.last_error() << '\n';
        rc = 1;
      }
    }
    else
    {
      usage();
      rc = 1;
    }
  }
  catch (const std::exception & e)
  {
    std::cerr << "bad argument: " << e.what() << '\n';
    rc = 1;
  }

  bus.close();
  return rc;
}
