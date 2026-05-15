#include <unistd.h>
#include <string>
#include <vector>

#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <sbgCom/sbgCom.h>

// State interface names follow the ros2_control IMUSensor semantic component convention
// so any imu_sensor_broadcaster can read them without additional mapping.
//
// Data is in NED (North-East-Down) frame — same as the standalone sbg_node.
// TODO: validate NED→ENU rotation matrix and apply it here (see sbg_node.cpp).

namespace caddy_ai2_ros2_control_sensors_sbg_ig500n
{

class SbgHardwareInterface : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override
  {
    if (SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto & p = info.hardware_parameters;
    if (p.count("port"))      port_      = p.at("port");
    if (p.count("baudrate"))  baudrate_  = std::stoi(p.at("baudrate"));
    if (p.count("frequency")) frequency_ = std::stoi(p.at("frequency"));

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &) override
  {
    last_error_ = sbgComInit(port_.c_str(), baudrate_, &protocol_handle_);
    if (check_error("sbgComInit")) return hardware_interface::CallbackReturn::ERROR;

    usleep(50 * 1000);

    last_error_ = sbgSetDefaultOutputMask(protocol_handle_, OUTPUT_MASK_);
    if (check_error("sbgSetDefaultOutputMask")) return hardware_interface::CallbackReturn::ERROR;

    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONTINUOUS_MODE_ENABLE, 1);
    if (check_error("sbgSetContinuousMode")) return hardware_interface::CallbackReturn::ERROR;

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &) override
  {
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &) override
  {
    active_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &) override
  {
    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONT_TRIGGER_MODE_DISABLE, 1);
    check_error("sbgSetContinuousMode: disable");
    last_error_ = sbgProtocolClose(protocol_handle_);
    check_error("sbgProtocolClose");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> ifaces;

    ifaces.emplace_back("imu_sensor", "orientation.x",          &orientation_[0]);
    ifaces.emplace_back("imu_sensor", "orientation.y",          &orientation_[1]);
    ifaces.emplace_back("imu_sensor", "orientation.z",          &orientation_[2]);
    ifaces.emplace_back("imu_sensor", "orientation.w",          &orientation_[3]);
    ifaces.emplace_back("imu_sensor", "angular_velocity.x",     &angular_velocity_[0]);
    ifaces.emplace_back("imu_sensor", "angular_velocity.y",     &angular_velocity_[1]);
    ifaces.emplace_back("imu_sensor", "angular_velocity.z",     &angular_velocity_[2]);
    ifaces.emplace_back("imu_sensor", "linear_acceleration.x",  &linear_acceleration_[0]);
    ifaces.emplace_back("imu_sensor", "linear_acceleration.y",  &linear_acceleration_[1]);
    ifaces.emplace_back("imu_sensor", "linear_acceleration.z",  &linear_acceleration_[2]);

    ifaces.emplace_back("navsat_sensor", "latitude",  &latitude_);
    ifaces.emplace_back("navsat_sensor", "longitude", &longitude_);
    ifaces.emplace_back("navsat_sensor", "altitude",  &altitude_);

    return ifaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time &, const rclcpp::Duration &) override
  {
    if (!active_) return hardware_interface::return_type::OK;

    sbgProtocolContinuousModeHandle(protocol_handle_);

    SbgOutput output{};
    sbgGetDefaultOutput(protocol_handle_, &output);

    if (!output.outputMask) return hardware_interface::return_type::OK;

    if (output.outputMask & IMU_OUTPUT_MASK_) {
      tf2::Matrix3x3 mat(
        output.stateMatrix[0], output.stateMatrix[1], output.stateMatrix[2],
        output.stateMatrix[3], output.stateMatrix[4], output.stateMatrix[5],
        output.stateMatrix[6], output.stateMatrix[7], output.stateMatrix[8]
      );
      tf2::Quaternion q;
      mat.getRotation(q);

      orientation_[0] = q.x();
      orientation_[1] = q.y();
      orientation_[2] = q.z();
      orientation_[3] = q.w();

      angular_velocity_[0] = output.gyroscopes[0];
      angular_velocity_[1] = output.gyroscopes[1];
      angular_velocity_[2] = output.gyroscopes[2];

      linear_acceleration_[0] = output.accelerometers[0];
      linear_acceleration_[1] = output.accelerometers[1];
      linear_acceleration_[2] = output.accelerometers[2];
    }

    if (output.outputMask & GPS_OUTPUT_MASK_) {
      latitude_  = output.position[0];
      longitude_ = output.position[1];
      altitude_  = output.position[2];
    }

    return hardware_interface::return_type::OK;
  }

private:
  static constexpr int IMU_OUTPUT_MASK_ =
    SBG_OUTPUT_MATRIX | SBG_OUTPUT_GYROSCOPES | SBG_OUTPUT_ACCELEROMETERS;
  static constexpr int GPS_OUTPUT_MASK_ =
    SBG_OUTPUT_POSITION | SBG_OUTPUT_NAV_ACCURACY | SBG_OUTPUT_GPS_INFO;
  static constexpr int OUTPUT_MASK_ = IMU_OUTPUT_MASK_ | GPS_OUTPUT_MASK_;

  std::string port_{"dev/sbg"};
  int baudrate_{921600};
  int frequency_{500};

  SbgProtocolHandle protocol_handle_{};
  SbgErrorCode last_error_{SBG_NO_ERROR};
  bool active_{false};

  double orientation_[4]{0.0, 0.0, 0.0, 1.0};  // x y z w — NED frame
  double angular_velocity_[3]{0.0, 0.0, 0.0};
  double linear_acceleration_[3]{0.0, 0.0, 0.0};

  double latitude_{0.0};
  double longitude_{0.0};
  double altitude_{0.0};

  bool check_error(const std::string & msg)
  {
    if (last_error_ != SBG_NO_ERROR) {
      RCLCPP_ERROR(rclcpp::get_logger("SbgHardwareInterface"),
        "SBG error at '%s': code %d", msg.c_str(), static_cast<int>(last_error_));
      return true;
    }
    return false;
  }
};

}  // namespace caddy_ai2_ros2_control_sensors_sbg_ig500n

PLUGINLIB_EXPORT_CLASS(
  caddy_ai2_ros2_control_sensors_sbg_ig500n::SbgHardwareInterface,
  hardware_interface::SensorInterface
)
