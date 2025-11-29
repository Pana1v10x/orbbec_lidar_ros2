#include <utility>

#include "orbbec_lidar_ros2/ob_lidar_driver.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace ob_lidar {
OBLidarDriver::OBLidarDriver(const rclcpp::NodeOptions& options)
    : Node("orbbec_camera_driver", "/", options),
      logger_(this->get_logger()),
      use_intra_process_(options.use_intra_process_comms()) {
  init();
}
OBLidarDriver::OBLidarDriver(const std::string& node_name, const std::string& ns,
                             const rclcpp::NodeOptions& node_options)
    : Node(node_name, ns, node_options),
      logger_(this->get_logger()),
      use_intra_process_(node_options.use_intra_process_comms()) {
  init();
}

void OBLidarDriver::init() {
  log_level_ = this->declare_parameter("log_level", "info");
  config_file_ = this->declare_parameter("config_file", "");
  parameters_ = std::make_shared<Parameters>(this);

  // Try to create device from ROS parameters first, fallback to config_file if provided
  std::string device_name = this->declare_parameter("device_name", "");
  std::string device_type_str = this->declare_parameter("device_type", "");

  RCLCPP_INFO(logger_, "device_name: '%s', device_type: '%s', config_file: '%s'",
              device_name.c_str(), device_type_str.c_str(), config_file_.c_str());

  if (!device_name.empty() && !device_type_str.empty()) {
    // Create device from ROS parameters
    LidarType lidar_type = LidarType::SINGLE_LINE;
    if (device_type_str == "single_line") {
      lidar_type = LidarType::SINGLE_LINE;
    } else if (device_type_str == "multi_line") {
      lidar_type = LidarType::MULTI_LINE;
    } else {
      RCLCPP_WARN(logger_, "Unknown device_type '%s', defaulting to SINGLE_LINE", device_type_str.c_str());
    }

    // Read network configuration
    std::string ip = this->declare_parameter("network.ip", "192.168.1.120");
    std::string protocol_str = this->declare_parameter("network.protocol", "udp");
    uint16_t port = this->declare_parameter("network.port", 2228);

    LidarProtocolType protocol_type = LidarProtocolType::UDP;
    if (protocol_str == "udp" || protocol_str == "UDP") {
      protocol_type = LidarProtocolType::UDP;
    } else if (protocol_str == "tcp" || protocol_str == "TCP") {
      protocol_type = LidarProtocolType::TCP;
    } else {
      RCLCPP_WARN(logger_, "Unknown protocol '%s', defaulting to UDP", protocol_str.c_str());
    }

    // Create NetworkConfig
    auto network_config = std::make_shared<NetworkConfig>(lidar_type, protocol_type, ip, port);
    // Note: single_port_mode is handled internally by NetworkConfig based on lidar_type

    // Read logging configuration
    int max_log_file_size = this->declare_parameter("logging.max_file_size", 1048576);
    int max_log_file_num = this->declare_parameter("logging.max_file_num", 1);
    std::string log_file_dir = this->declare_parameter("logging.log_file_dir", "/root/.ros/log");

    // Create LoggerConfig (logging flags use defaults from constructor)
    auto logger_config = std::make_shared<LoggerConfig>(
        static_cast<uint32_t>(max_log_file_size),
        static_cast<uint32_t>(max_log_file_num),
        log_file_dir);

    // Create DeviceConfig
    device_config_ = std::make_shared<DeviceConfig>(device_name, network_config, logger_config);
    device_ = DeviceFactory::create(device_config_);
  } else if (!config_file_.empty()) {
    // Fallback to config file if provided
    device_ = DeviceFactory::create(config_file_);
  } else {
    RCLCPP_ERROR(logger_, "Neither device_name/device_type parameters nor config_file is set");
    exit(-1);
  }

  if (device_ == nullptr) {
    RCLCPP_ERROR(logger_, "Failed to create device");
    exit(-1);
  }
  device_info_ = device_->getInfo();
  orbbec_lidar_node_ =
      std::make_unique<OrbbecLidarNode>(this, device_, parameters_, use_intra_process_);
}

OBLidarDriver::~OBLidarDriver() = default;

}  // namespace ob_lidar

RCLCPP_COMPONENTS_REGISTER_NODE(ob_lidar::OBLidarDriver)
