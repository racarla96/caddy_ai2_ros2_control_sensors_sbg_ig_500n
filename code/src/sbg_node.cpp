#include <rclcpp/rclcpp.hpp>
#include <unistd.h> // para Linux 
#include <chrono>
#include <sbgCom/sbgCom.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

using namespace std;

class SBGNode : public rclcpp::Node {
private:
  string port = "/dev/sbg";
  int baudrate = 921600;
  string imu_frame_id = "imu_link";
  string imu_frame_ned_id = imu_frame_id + "_ned";
  string navsat_frame_id = "navsat_link";
  int frequency = 500;

  SbgOutput pOutput;
  SbgProtocolHandle protocol_handle_; 
  SbgErrorCode last_error_;

  tf2::Vector3 _vec;
  tf2::Matrix3x3 IMU2ROS;
  tf2::Matrix3x3 NED2ENU;
  tf2::Matrix3x3 _output_matrix;
  tf2::Quaternion _quat;
  tf2::Matrix3x3 _aux;

  double gravity = 9.81;

  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
  //orientation, angular_velocity, linear_acceleration
  const double IMU_COVARIANCES[3] = {0.0174532925, 0.00872664625, 0.049};
  std::shared_ptr<sensor_msgs::msg::Imu> imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
  std::shared_ptr<sensor_msgs::msg::Imu> imu_ned_msg = std::make_shared<sensor_msgs::msg::Imu>();
  const int IMU_OUTPUT_MASK = SBG_OUTPUT_MATRIX | 
                              SBG_OUTPUT_GYROSCOPES |
                              SBG_OUTPUT_ACCELEROMETERS;

  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatStatus.html
  // https://docs.ros2.org/foxy/api/sensor_msgs/msg/NavSatFix.html
  std::shared_ptr<sensor_msgs::msg::NavSatFix> navsat_msg = std::make_shared<sensor_msgs::msg::NavSatFix>();
  const int GPS_OUTPUT_MASK = SBG_OUTPUT_POSITION |
                              SBG_OUTPUT_NAV_ACCURACY |
                              SBG_OUTPUT_GPS_INFO;  

  const int OUTPUT_MASK = IMU_OUTPUT_MASK |
                          GPS_OUTPUT_MASK;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_ned_pub;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr navsat_pub;
  rclcpp::TimerBase::SharedPtr timer_;

  void periodicTask() {
    sbgProtocolContinuousModeHandle(protocol_handle_);
    sbgGetDefaultOutput(protocol_handle_, &pOutput);

    if (pOutput.outputMask)
    {
      if (pOutput.outputMask & IMU_OUTPUT_MASK) {
        imu_msg->header.stamp = this->now();
        imu_msg->header.frame_id = imu_frame_id;

        imu_ned_msg->header.stamp = this->now();
        imu_ned_msg->header.frame_id = imu_frame_ned_id;

        // The SBG IG-500N outputs data in NED (North-East-Down) frame.
        // /imu_ned publishes raw NED data for reference.
        // /imu should publish in REP-103 (ENU, x-forward, y-left, z-up) but the
        // IMU2ROS transform block is pending validation — kept commented until confirmed.
        // TODO: validate IMU2ROS rotation matrix against real vehicle axes and enable.
        _output_matrix.setValue(pOutput.stateMatrix[0], pOutput.stateMatrix[1], pOutput.stateMatrix[2],
                      pOutput.stateMatrix[3], pOutput.stateMatrix[4], pOutput.stateMatrix[5],
                      pOutput.stateMatrix[6], pOutput.stateMatrix[7], pOutput.stateMatrix[8]);

        _output_matrix.getRotation(_quat);

        imu_ned_msg->orientation.w = _quat.w();
        imu_ned_msg->orientation.x = _quat.x();
        imu_ned_msg->orientation.y = _quat.y();
        imu_ned_msg->orientation.z = _quat.z();

        imu_ned_msg->angular_velocity.x = pOutput.gyroscopes[0];
        imu_ned_msg->angular_velocity.y = pOutput.gyroscopes[1];
        imu_ned_msg->angular_velocity.z = pOutput.gyroscopes[2];

        imu_ned_msg->linear_acceleration.x = pOutput.accelerometers[0];
        imu_ned_msg->linear_acceleration.y = pOutput.accelerometers[1];
        imu_ned_msg->linear_acceleration.z = pOutput.accelerometers[2];

        imu_pub->publish(*imu_msg);
        imu_ned_pub->publish(*imu_ned_msg);
      }
      if (pOutput.outputMask & GPS_OUTPUT_MASK) {
        navsat_msg->header.stamp = this->now();
        navsat_msg->header.frame_id = navsat_frame_id;

        navsat_msg->latitude  = pOutput.position[0];
        navsat_msg->longitude = pOutput.position[1];
        navsat_msg->altitude  = pOutput.position[2];

        for (int i = 0; i < 9; i++)
            navsat_msg->position_covariance[i] = 0;

        // covariance_type: 0=unknown, 1=approximated, 2=diagonal_known, 3=known
        navsat_msg->position_covariance_type = 1;
        for (int i = 0; i <= 8; i += 4)
            navsat_msg->position_covariance[i] = pOutput.positionAccuracy;

        navsat_msg->status.status  = (pOutput.gpsFlags & 0x03) ? 0 : -1;
        navsat_msg->status.service = 1; // GPS

        navsat_pub->publish(*navsat_msg);
      }
    }    
  }

  bool checkError(const string & msg) {
    if (last_error_ != SBG_NO_ERROR) {
      RCLCPP_ERROR(this->get_logger(), "SBG error at '%s': code %d", msg.c_str(), static_cast<int>(last_error_));
      return true;
    }
    return false;
  }

public:
  SBGNode(const rclcpp::NodeOptions &options) : Node("sbg_node", options){
    this->declare_parameter("port", port);
    this->get_parameter("port", port);

    this->declare_parameter("baudrate", baudrate);
    this->get_parameter("baudrate", baudrate);

    this->declare_parameter("imu_frame_id", imu_frame_id);
    this->get_parameter("imu_frame_id", imu_frame_id);
    imu_frame_ned_id = imu_frame_id + "_ned";

    this->declare_parameter("navsat_frame_id", navsat_frame_id);
    this->get_parameter("navsat_frame_id", navsat_frame_id);

    this->declare_parameter("frequency", frequency);
    this->get_parameter("frequency", frequency);

    last_error_ = sbgComInit(port.c_str(), baudrate, &protocol_handle_);
    if(checkError("sbgComInit")) return;
    usleep(50*1000);    // time_period en microsegundos

    last_error_ = sbgSetDefaultOutputMask(protocol_handle_, OUTPUT_MASK);
    if(checkError("sbgSetDefaultOutputMask")) return;

    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONTINUOUS_MODE_ENABLE, 1);
    if(checkError("sbgSetContinuousMode: SBG_CONTINUOUS_MODE_ENABLE")) return;

    imu_pub     = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
    imu_ned_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu_ned", 1);
    navsat_pub  = this->create_publisher<sensor_msgs::msg::NavSatFix>("navsat", 1);
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / frequency), std::bind(&SBGNode::periodicTask, this));

    // Constants values
    for (int i = 0; i < 9; i++) {
        imu_msg->orientation_covariance[i] = 0;
        imu_msg->angular_velocity_covariance[i] = 0;
        imu_msg->linear_acceleration_covariance[i] = 0;
    }
    
    for (int i = 0; i < 9; i+=3) {
        imu_msg->orientation_covariance[i] = IMU_COVARIANCES[0];
        imu_msg->angular_velocity_covariance[i] = IMU_COVARIANCES[1];
        imu_msg->linear_acceleration_covariance[i] = IMU_COVARIANCES[2];
    }

    for (int i = 0; i < 9; i++) {
        imu_ned_msg->orientation_covariance[i] = 0;
        imu_ned_msg->angular_velocity_covariance[i] = 0;
        imu_ned_msg->linear_acceleration_covariance[i] = 0;
    }
    
    for (int i = 0; i < 9; i+=3) {
        imu_ned_msg->orientation_covariance[i] = IMU_COVARIANCES[0];
        imu_ned_msg->angular_velocity_covariance[i] = IMU_COVARIANCES[1];
        imu_ned_msg->linear_acceleration_covariance[i] = IMU_COVARIANCES[2];
    }

    IMU2ROS.setValue(1.0,  0.0,  0.0,
                     0.0, -1.0,  0.0,
                     0.0,  0.0, -1.0);

    NED2ENU.setValue(0.0,  1.0,  0.0,
                     1.0,  0.0,  0.0,
                     0.0,  0.0, -1.0);

    RCLCPP_INFO(this->get_logger(), "SBG node started");
  }

  // Destructor
  ~SBGNode() {
    last_error_ = sbgSetContinuousMode(protocol_handle_, SBG_CONT_TRIGGER_MODE_DISABLE, 1);
    if(checkError("sbgSetContinuousMode: SBG_CONT_TRIGGER_MODE_DISABLE")) return;

    last_error_ = sbgProtocolClose(protocol_handle_);
    if(checkError("sbgProtocolClose")) return;

    RCLCPP_INFO(this->get_logger(), "SBG node destroyed");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<SBGNode>(options));
  rclcpp::shutdown();
  return 0;   
}