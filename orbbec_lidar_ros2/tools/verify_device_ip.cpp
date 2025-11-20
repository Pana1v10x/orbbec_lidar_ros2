#include <iostream>
#include <string>
#include "orbbec_lidar/orbbec_lidar.hpp"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <device_ip> [port]" << std::endl;
    std::cout << "Example: " << argv[0] << " 192.168.1.200 2228" << std::endl;
    return 1;
  }

  std::string device_ip = argv[1];
  uint16_t port = (argc >= 3) ? static_cast<uint16_t>(std::stoi(argv[2])) : 2228;

  try {
    ob_lidar::DeviceConfigBuilder builder;
    builder.setDeviceName("verify_device")
           .setType("single_line")
           .setProtocolType("udp")
           .setIP(device_ip)
           .setPort(port);
    
    auto config = builder.build();
    std::cout << "Connecting to device at " << device_ip << ":" << port << "..." << std::endl;
    auto device = ob_lidar::DeviceFactory::create(config);
    
    if (device == nullptr) {
      std::cerr << "ERROR: Failed to connect to device" << std::endl;
      return 1;
    }

    std::string current_ip = ob_lidar::getIPAddress(device);
    std::cout << "Successfully connected!" << std::endl;
    std::cout << "Device hardware IP address: " << current_ip << std::endl;
    
    if (current_ip == device_ip) {
      std::cout << "✓ IP address matches expected value!" << std::endl;
    } else {
      std::cout << "⚠ WARNING: IP address mismatch!" << std::endl;
      std::cout << "  Expected: " << device_ip << std::endl;
      std::cout << "  Actual:   " << current_ip << std::endl;
    }

  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

