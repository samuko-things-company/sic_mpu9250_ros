#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "sic_mpu9250_ros2/sic_mpu9250_cppserial_lib.hpp"



void delay_ms(unsigned long milliseconds) {
  usleep(milliseconds*1000);
}



class SicMPU9250Node : public rclcpp::Node
{
public:
    SicMPU9250Node() : Node("sic_mpu9250")
    {
      /*---------------node parameter declaration-----------------------------*/
      this->declare_parameter<std::string>("frame_id", "imu");
      this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
      this->declare_parameter<double>("publish_frequency", 10.0);
      this->declare_parameter<bool>("publish_tf_on_map_frame", false);

      frame_id = this->get_parameter("frame_id").as_string();
      RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id.c_str());

      port = this->get_parameter("port").as_string();
      RCLCPP_INFO(this->get_logger(), "port: %s", port.c_str());

      publish_frequency = this->get_parameter("publish_frequency").as_double();
      RCLCPP_INFO(this->get_logger(), "publish_frequency: %f", publish_frequency);

      publish_tf_on_map_frame = this->get_parameter("publish_frequency").as_bool();
      RCLCPP_INFO(this->get_logger(), "publish_tf_on_map_frame: %f", publish_tf_on_map_frame);
      /*---------------------------------------------------------------------*/


      /*----------start connection to sic_mpu9250_driver module---------------*/
      sic_mpu9250.connect(port);
      // wait for the imu to fully setup
      for (int i=1; i<=10; i+=1){ 
        delay_ms(1000);
        RCLCPP_INFO(this->get_logger(), "%d", i);
      }
      /*---------------------------------------------------------------------*/


      /*----------initialize IMU message---------------*/
      messageImu.header.frame_id = frame_id;
      
      sic_mpu9250.getRPYVariance(data_x, data_y, data_z);
      messageImu.orientation_covariance = { data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z };

      sic_mpu9250.getRPYrateVariance(data_x, data_y, data_z);
      messageImu.angular_velocity_covariance = { data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z };

      sic_mpu9250.getAccVariance(data_x, data_y, data_z);
      messageImu.linear_acceleration_covariance = { data_x, 0.0, 0.0, 0.0, data_y, 0.0, 0.0, 0.0, data_z };
      /*---------------------------------------------------------------------*/


      /*---------------start imu and mag publishers and timer-----------------------------*/
      imu_data_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
      imu_rpy_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("/imu/data_rpy", 10);

      timer_ = this->create_wall_timer(
        std::chrono::microseconds((long)(1000000/publish_frequency)),
        std::bind(&SicMPU9250Node::publish_imu_callback, this));
      /*---------------------------------------------------------------------*/

      RCLCPP_INFO(this->get_logger(), "sic_mpu9250 node has started");
    }

private:
  void publish_imu_callback(){  
    messageImu.header.stamp = rclcpp::Clock().now();

    sic_mpu9250.getQuat(data_w, data_x, data_y, data_z);
    messageImu.orientation.w = data_w;
    messageImu.orientation.x = data_x;
    messageImu.orientation.y = data_y;
    messageImu.orientation.z = data_z;

    sic_mpu9250.getRPYrate(data_x, data_y, data_z);
    messageImu.angular_velocity.x = data_x;
    messageImu.angular_velocity.y = data_y;
    messageImu.angular_velocity.z = data_z;

    sic_mpu9250.getAcc(data_x, data_y, data_z);
    messageImu.linear_acceleration.x = data_x;
    messageImu.linear_acceleration.y = data_y;
    messageImu.linear_acceleration.z = data_z;

    geometry_msgs::msg::Vector3Stamped rpy;
    tf2::Matrix3x3(tf2::Quaternion(
      messageImu.orientation.x,
      messageImu.orientation.y,
      messageImu.orientation.z,
      messageImu.orientation.w
    )).getRPY(rpy.vector.x, rpy.vector.y, rpy.vector.z);
    rpy.header = messageImu.header;

    imu_data_publisher_->publish(messageImu);
    imu_rpy_publisher_->publish(rpy);

    if (publish_imu_tf_on_map_frame) {
      publish_imu_tf_on_map_frame(messageImu);
    }
  }

  void publish_imu_tf_on_map_frame(sensor_msgs::msg::Imu messageImu){  
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = messageImu.header.stamp;

    transform.header.frame_id = "map";
    transform.child_frame_id = messageImu.header.frame_id;

    transform.transform.rotation.w = messageImu.orientation.w;
    transform.transform.rotation.x = messageImu.orientation.x;
    transform.transform.rotation.y = messageImu.orientation.y;
    transform.transform.rotation.z = messageImu.orientation.z;

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;

    tf_broadcaster_.sendTransform(transform);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_data_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_rpy_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Imu messageImu = sensor_msgs::msg::Imu();

  std::string frame_id;
  std::string port;
  double publish_frequency;
  bool publish_tf_on_map_frame;

  SIC sic_mpu9250;
  float data_w, data_x, data_y, data_z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SicMPU9250Node>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}