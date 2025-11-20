#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <cctype>
#include "orbbec_lidar/orbbec_lidar.hpp"

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <new_ip> [current_ip] [port]" << std::endl;
    std::cout << "       " << argv[0] << " <new_ip> [port]" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << argv[0] << " 192.168.1.200" << std::endl;
    std::cout << "    (Auto-discover device and set IP to 192.168.1.200)" << std::endl;
    std::cout << "  " << argv[0] << " 192.168.1.200 192.168.1.100" << std::endl;
    std::cout << "    (Connect to device at 192.168.1.100 and set IP to 192.168.1.200)" << std::endl;
    std::cout << "  " << argv[0] << " 192.168.1.200 192.168.1.100 2228" << std::endl;
    std::cout << "    (Same as above, with custom port)" << std::endl;
    std::cout << std::endl;
    std::cout << "This tool changes the hardware IP address of the lidar device." << std::endl;
    std::cout << "If current_ip is not provided, the tool will auto-discover devices." << std::endl;
    std::cout << "If multiple devices are found, you must specify the current_ip." << std::endl;
    return 1;
  }

  std::string new_ip = argv[1];
  std::string current_ip;
  uint16_t port = 2228;
  
  // Parse arguments: new_ip [current_ip] [port]
  if (argc >= 3) {
    // Check if third argument is a port number (all digits) or an IP address
    std::string arg3 = argv[2];
    bool is_port = true;
    for (char c : arg3) {
      if (!std::isdigit(c)) {
        is_port = false;
        break;
      }
    }
    
    if (is_port && arg3.length() <= 5) {
      // Third argument is a port number, so current_ip was not provided
      port = static_cast<uint16_t>(std::stoi(arg3));
    } else {
      // Third argument is an IP address (current_ip)
      current_ip = arg3;
      if (argc >= 4) {
        port = static_cast<uint16_t>(std::stoi(argv[3]));
      }
    }
  }

  // Auto-discover device if current_ip not provided
  if (current_ip.empty()) {
    std::cout << "Auto-discovering devices..." << std::endl;
    try {
      auto device_manager = std::make_shared<ob_lidar::DeviceManager>();
      // Wait a few seconds for device discovery
      std::this_thread::sleep_for(std::chrono::seconds(3));
      
      auto devices = device_manager->getDevices();
      
      if (devices.empty()) {
        std::cerr << "ERROR: No devices found. Please specify the current IP address:" << std::endl;
        std::cerr << "  " << argv[0] << " " << new_ip << " <current_ip> [port]" << std::endl;
        return 1;
      } else if (devices.size() > 1) {
        std::cerr << "ERROR: Multiple devices found. Please specify which device to use:" << std::endl;
        std::cerr << "Found devices:" << std::endl;
        for (size_t i = 0; i < devices.size(); ++i) {
          try {
            std::string device_ip = ob_lidar::getIPAddress(devices[i]);
            std::cerr << "  " << (i + 1) << ". " << device_ip << std::endl;
          } catch (...) {
            std::cerr << "  " << (i + 1) << ". (Unable to get IP)" << std::endl;
          }
        }
        std::cerr << std::endl;
        std::cerr << "Usage: " << argv[0] << " " << new_ip << " <current_ip> [port]" << std::endl;
        return 1;
      } else {
        // Exactly one device found
        try {
          current_ip = ob_lidar::getIPAddress(devices[0]);
          std::cout << "Found device at: " << current_ip << std::endl;
        } catch (const std::exception& e) {
          std::cerr << "ERROR: Failed to get IP address from discovered device: " << e.what() << std::endl;
          std::cerr << "Please specify the current IP address manually." << std::endl;
          return 1;
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "ERROR: Failed to discover devices: " << e.what() << std::endl;
      std::cerr << "Please specify the current IP address manually:" << std::endl;
      std::cerr << "  " << argv[0] << " " << new_ip << " <current_ip> [port]" << std::endl;
      return 1;
    }
  }

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

