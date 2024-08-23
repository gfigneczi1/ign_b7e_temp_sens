#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

class SensorNode : public rclcpp::Node
{
public:
  SensorNode() : Node("sensor_node")
  {
    temperature_pub = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 1);
    humidity_pub = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("relative_humidity", 1);
  }

  // publisher declaration
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_pub;

  // declare messages as member variables
  sensor_msgs::msg::RelativeHumidity relativeHumidity;
  sensor_msgs::msg::Temperature temperature;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SensorNode>();

  rclcpp::Rate rate(10);

  // emulation data: counting with different gradients
  double T = 0; // starting from 0 celsius
  double dT = 0.5; // 0.5 celsius steps
  double Tmax = 50; // maximum 50 celsius
  double H = 1; // starting from 100% rel humidity
  double dH = -0.01; // 1% steps
  double Hmin = 0; // 0% rel humidity

  while (rclcpp::ok())
  {
    T+=dT; H+=dH;
    if (T>=Tmax){T=0;}
    if (H<=Hmin){H=1;}
    node->temperature.temperature = T;
    node->relativeHumidity.relative_humidity = H;
    node->temperature_pub->publish(node->temperature);
    node->humidity_pub->publish(node->relativeHumidity);

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}