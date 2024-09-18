#include <iostream>
#include <chrono>
#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_whill/srv/set_speed_profile.hpp"
#include "ros2_whill/srv/set_power.hpp"

#define CONFIG_SIZE 9

using namespace std::chrono_literals;

/**
 * @brief
 * Mapping the keys of buttons and axes on xbox controller. The ruler of the key layout is according to "joy_node" which is a ROS Package.
 */
namespace JoyButtonKey
{
  /**
   * @brief
   * Key layout of Buttons
   */  
  namespace Buttons
  {
    const int A = 0;
    const int B = 1;
    const int X = 2;
    const int Y = 3;
    const int LB = 4;
    const int RB = 5;
    const int BACK = 6;
    const int START = 7;
    const int HOME = 8;
    const int LEFT_STICK = 9;
    const int RIGHT_STICK = 10;
  }

  /**
   * @brief
   * Key layout of Axes
   */  namespace Axes
  {
    const int LEFT_STICK_L_R = 0;
    const int LEFT_STICK_U_D = 1;
    const int LT = 2;
    const int RIGHT_STICK_L_R = 3;
    const int RIGHT_STICK_U_D = 4;
    const int RT = 5;
    const int ARROW_L_R = 6;
    const int ARROW_U_D = 7;
  }

};

int64_t sec_to_nano(double second)
{
  return int64_t(second * pow(10, 9));
}

using namespace JoyButtonKey;
class WhillJoy : public rclcpp::Node
{
  public:
    /**
     * @brief
     * A Constructor where to instantiate ROS API such as Parameter, Topic and Service and then initialize some parameters.
     * 
     * @param options NodeOptions Instance which has some optional setting about this node.
     */
    WhillJoy(const rclcpp::NodeOptions &options) : Node("whill_joy", options)
    {
      this->frequency_ = this->declare_parameter("frequency", 10);
      this->pressed_duration_ = this->declare_parameter("pressed_duration", 2.0);
      // Initial speed profile
      this->speed_profile_initial_ = this->declare_parameter<std::vector<int64_t>>(
        "speed_profile_initial",
        std::vector<int64_t>{30, 16, 82, 20, 16, 64, 15, 56, 72}
      );
      
      // Subscriber
      this->joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy_in", 10, std::bind(&WhillJoy::ros_joy_callback_, this, std::placeholders::_1));

      // Publisher
      this->joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("joy_out", rclcpp::QoS(10));

      // Client
      this->set_speed_profile_client_ = this->create_client<ros2_whill::srv::SetSpeedProfile>("set_speed_profile_srv");
      this->set_power_client_ = this->create_client<ros2_whill::srv::SetPower>("set_power_srv");

      this->set_speed_profile_client_->wait_for_service();
      this->set_power_client_->wait_for_service();

      initial_speed_profiles_();
      set_speed_();
    }

  private:
    //
    // ROS2 Objects
    //
    
    // Parameters
    long unsigned int frequency_;
    double pressed_duration_;

    // Parameters
    bool power_on_ = false;
    ros2_whill::srv::SetSpeedProfile::Request::SharedPtr speed_profile_;
    std::vector<int64_t> speed_profile_initial_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

    // Client
    rclcpp::Client<ros2_whill::srv::SetSpeedProfile>::SharedPtr set_speed_profile_client_;
    rclcpp::Client<ros2_whill::srv::SetPower>::SharedPtr set_power_client_;

    void ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy);
    void set_speed_();
    void set_power_();
    void initial_speed_profiles_();    
    void stop_pub_();
};

void WhillJoy::ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if(joy->axes.size() < 8 || joy->buttons.size() < 11) return;

  // If Axis Up/Down is pressed
  if (joy->axes[Axes::ARROW_U_D] > 0.0)
  {
    speed_profile_->fm1 += 5;
    set_speed_();
  }
  else if (joy->axes[Axes::ARROW_U_D] < 0.0)
  {
    speed_profile_->fm1 -= 5;
    set_speed_();
  }

  // If Axis Left/Right is pressed
  if (joy->axes[Axes::ARROW_L_R] < 0.0)
  {
    speed_profile_ ->tm1 += 5;
    set_speed_();
  }
  else if (joy->axes[Axes::ARROW_L_R] > 0.0)
  {
     speed_profile_ -> tm1 -= 5;
     set_speed_();
  }
  

  // If button HOME is pressed
  // if (joy->buttons[Buttons::HOME] == 1)
  // {
  //   set_power_();
  // }

  sensor_msgs::msg::Joy joy_msg;
  joy_msg.axes = joy->axes;
  joy_msg.buttons = joy->buttons;
  joy_msg.header = joy->header;

  if (joy->buttons[Buttons::LEFT_STICK] == 1)
  {
    joy_msg.axes[Axes::LEFT_STICK_L_R] = 0.0;
    joy_msg.axes[Axes::LEFT_STICK_U_D] = 0.0;

    this->joy_pub_->publish(joy_msg);
  }
  else if (get_clock()->now().seconds() - (joy->header.stamp.sec + joy->header.stamp.nanosec * pow(10, -9)) < 0.05)
  {
    this->joy_pub_->publish(joy_msg);
  }

}

void WhillJoy::set_speed_()
{

  if (speed_profile_->fm1 >= 60)
  {
    speed_profile_->fm1 = 60;
    RCLCPP_WARN(get_logger(), "Speed Forward is set to Maximum!!!");
  }
  if (speed_profile_->fm1 <= 8)
  {
    speed_profile_->fm1 = 8; 
    RCLCPP_WARN(get_logger(), "Speed Forward is set to Minimum!!!");
  }
  if (speed_profile_->tm1 >= 35)
  {
    speed_profile_->fm1 = 35;
    RCLCPP_WARN(get_logger(), "Speed Turn is set to Maximum!!!");
  }
  if (speed_profile_->tm1 <= 8)
  {
    speed_profile_->tm1 = 8;
    RCLCPP_WARN(get_logger(), "Speed Turn is set to Minimum!!!");
  }

  auto req_future = this->set_speed_profile_client_->async_send_request(speed_profile_);

  float l_scale, a_scale;

  l_scale = speed_profile_->fm1 * 0.1f * 1000 / 3600;
  a_scale = speed_profile_->tm1 * 0.1f * 1000 / 3600 / 0.225;

  RCLCPP_INFO(this->get_logger(), "Linear Velocity Maximum: %f m/s, Angular velocity Maximum: %f rad/s", l_scale, a_scale);
  
  rclcpp::sleep_for(std::chrono::nanoseconds(sec_to_nano(pressed_duration_)));
}

void WhillJoy::initial_speed_profiles_()
{
  speed_profile_ = std::make_shared<ros2_whill::srv::SetSpeedProfile::Request>();

  speed_profile_->fm1 = speed_profile_initial_[0];
  speed_profile_->fa1 = speed_profile_initial_[1];
  speed_profile_->fd1 = speed_profile_initial_[2];
  speed_profile_->rm1 = speed_profile_initial_[3];
  speed_profile_->ra1 = speed_profile_initial_[4];
  speed_profile_->rd1 = speed_profile_initial_[5];
  speed_profile_->tm1 = speed_profile_initial_[6];
  speed_profile_->ta1 = speed_profile_initial_[7];
  speed_profile_->td1 = speed_profile_initial_[8];
  speed_profile_->s1 = 4;

  float l_scale, a_scale;

  l_scale = speed_profile_->fm1 * 0.1f * 1000 / 3600;
  a_scale = speed_profile_->tm1 * 0.1f * 1000 / 3600 / 0.225;

  RCLCPP_INFO(this->get_logger(), "Linear Velocity Maximum: %f m/s, Angular velocity Maximum: %f rad/s", l_scale, a_scale);
}

void WhillJoy::set_power_()
{
  if (power_on_ == true)
  {
    auto req = std::make_shared<ros2_whill::srv::SetPower::Request>();
    req->p0 = 0;
    set_power_client_->async_send_request(req);
    power_on_ = false;
    RCLCPP_INFO(get_logger(), "Set Whill Power to OFF!!!");
  }
  else
  {
    auto req = std::make_shared<ros2_whill::srv::SetPower::Request>();
    req->p0 = 1;
    set_power_client_->async_send_request(req);
    power_on_ = true;
    RCLCPP_INFO(get_logger(), "Set Whill Power to ON!!!");
  }

  rclcpp::sleep_for(std::chrono::nanoseconds(sec_to_nano(pressed_duration_)));
}

int main(int argc, char *argv[])
{
    // ROS setup ROS2をセットアップ
    rclcpp::init(argc, argv);  // argc:与えの数、argv:与えの内容（文字列型の配列）

    // Nodeのオプションを設定するためのオブジェクトを宣言する
    rclcpp::NodeOptions options;
    // 宣言されていないパラメータは設定できないようにする。このオプションはtrueの場合、宣言されていないパラメータでも直接に設定できる
    options.allow_undeclared_parameters(false);
    options.automatically_declare_parameters_from_overrides(false);
    
    // プログラムの処理を終わらないようにするため、プロセスをスピンさせる、引数は：Nodeのオブジェクトのポインター
    rclcpp::spin(std::make_shared<WhillJoy>(options));

    // Nodeをシャットダウンする
    rclcpp::shutdown();

    return 0;
}