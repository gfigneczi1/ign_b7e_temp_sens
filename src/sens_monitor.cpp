#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;

class SensorMonitor : public rclcpp::Node
{
public:
  SensorMonitor() : Node("sensor_monitor")
  {
    temperature_sub = this->create_subscription<sensor_msgs::msg::Temperature>(
        "temperature", 10, std::bind(&SensorMonitor::temperature_callback, this, std::placeholders::_1));
    humidity_sub = this->create_subscription<sensor_msgs::msg::RelativeHumidity>(
        "relative_humidity", 10, std::bind(&SensorMonitor::humidity_callback, this, std::placeholders::_1));

    warning_pub = this->create_publisher<std_msgs::msg::String>("warning", 1);
  }

  void temperature_callback(const sensor_msgs::msg::Temperature& temperature)
  {
    temperatureWarning = false;
    if (temperature.temperature >= Tmin)
    {
      // warning to be set
      temperatureWarning = true;
    }
  }

  void humidity_callback(const sensor_msgs::msg::RelativeHumidity& humidity)
  {
    humidityWarning = false;
    if(humidity.relative_humidity >= Hmin)
    {
      humidityWarning = true;
    }
  }

  // publisher declaration
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr warning_pub;
  // subscriber declaration
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_sub;
  rclcpp::Subscription<sensor_msgs::msg::RelativeHumidity>::SharedPtr humidity_sub;
  // declare the warning message as member variable
  std_msgs::msg::String warning;

  // member to check warning type
  bool temperatureWarning = false;
  bool humidityWarning = false;

  // parameters
  double Tmin = 30;
  double Hmin = 0.8;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SensorMonitor>();

  rclcpp::Rate rate(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    if (node->temperatureWarning && node->humidityWarning)
    {
      node->warning.data = "Temperature and Humidity are too high!";
      node->warning_pub->publish(node->warning);
    }
    else if(node->temperatureWarning)
    {
      node->warning.data = "Temperature is too high!";
      node->warning_pub->publish(node->warning);
    }
    else if(node->humidityWarning)
    {
      node->warning.data = "Humidity is too high!";
      node->warning_pub->publish(node->warning);
    }
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}