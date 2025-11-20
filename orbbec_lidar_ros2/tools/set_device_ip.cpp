#include <iostream>
#include <string>
#include "orbbec_lidar/orbbec_lidar.hpp"

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <current_ip> <new_ip> [port]" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << argv[0] << " 192.168.1.100 192.168.1.200" << std::endl;
    std::cout << "    (Connect to device at 192.168.1.100 and set IP to 192.168.1.200)" << std::endl;
    std::cout << "  " << argv[0] << " 192.168.1.100 192.168.1.200 2228" << std::endl;
    std::cout << "    (Same as above, with custom port)" << std::endl;
    std::cout << std::endl;
    std::cout << "This tool changes the hardware IP address of the lidar device." << std::endl;
    std::cout << "You must connect to the device at its current IP first." << std::endl;
    std::cout << std::endl;
    std::cout << "To find the current IP address of your device, use:" << std::endl;
    std::cout << "  ros2 run orbbec_lidar_ros2 list_devices_node" << std::endl;
    return 1;
  }

  std::string current_ip = argv[1];
  std::string new_ip = argv[2];
  uint16_t port = (argc >= 4) ? static_cast<uint16_t>(std::stoi(argv[3])) : 2228;

  std::cout << "Changing device hardware IP address:" << std::endl;
  std::cout << "  Current IP: " << current_ip << std::endl;
  std::cout << "  New IP:     " << new_ip << std::endl;
  std::cout << "  Port:       " << port << std::endl;
  std::cout << std::endl;

  try {
    // Create device config to connect to current IP
    ob_lidar::DeviceConfigBuilder builder;
    builder.setDeviceName("temp_device")
           .setType("single_line")
           .setProtocolType("udp")
           .setIP(current_ip)
           .setPort(port);
    
    auto config = builder.build();
    std::cout << "Connecting to device at " << current_ip << "..." << std::endl;
    auto device = ob_lidar::DeviceFactory::create(config);
    
    if (device == nullptr) {
      std::cerr << "ERROR: Failed to connect to device at " << current_ip << std::endl;
      std::cerr << "Make sure the device is powered on and accessible at this IP." << std::endl;
      return 1;
    }

    std::cout << "Connected successfully!" << std::endl;
    std::cout << "Current device IP: " << ob_lidar::getIPAddress(device) << std::endl;
    std::cout << std::endl;

    // Set new IP address
    std::cout << "Setting new IP address to " << new_ip << "..." << std::endl;
    auto status = ob_lidar::setIPAddress(device, new_ip);
    
    if (status != ob_lidar::Status::OK) {
      std::cerr << "ERROR: Failed to set IP address. Status: " << static_cast<int>(status) << std::endl;
      return 1;
    }

    std::cout << "IP address set successfully!" << std::endl;
    
    // Verify the new IP
    std::string verify_ip = ob_lidar::getIPAddress(device);
    std::cout << "Verified device IP: " << verify_ip << std::endl;
    
    // Apply configs to save to flash memory
    std::cout << std::endl;
    std::cout << "Saving configuration to device flash memory..." << std::endl;
    status = ob_lidar::setApplyConfigs(device, 1);
    
    if (status == ob_lidar::Status::OK) {
      std::cout << "Configuration saved successfully!" << std::endl;
    } else {
      std::cerr << "WARNING: Failed to save configuration. Status: " << static_cast<int>(status) << std::endl;
      std::cerr << "The IP change may not persist after device reboot." << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "IP address change complete!" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "IMPORTANT: The device may need to be rebooted" << std::endl;
    std::cout << "for the new IP address (" << new_ip << ") to take effect." << std::endl;
    std::cout << "After reboot, update your config file to use:" << std::endl;
    std::cout << "  ip = \"" << new_ip << "\"" << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "ERROR: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}

