#include <iostream>
#include <chrono>
#include <vector>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_whill_interfaces/msg/whill_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_power.hpp"

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
      this->speed_step_ = this->declare_parameter("speed_step", 0);
      this->frequency_ = this->declare_parameter("frequency", 10);
      this->pressed_duration_ = this->declare_parameter("pressed_duration", 2.0);

      // Speed Profile Matrix
      this->speed_config_vector_ = strToInt(this->declare_parameter<std::vector<std::string>>(
        "speed_config_vector", std::vector<std::string>{
          "10, 16, 48, 10, 16, 40, 8, 56, 72",
          "15, 16, 64, 10, 16, 56, 10, 56, 72",
          "30, 16, 82, 20, 16, 64, 15, 56, 72",
          "45, 16, 90, 20, 24, 64, 18, 56, 72",
          "57, 16, 90, 20, 24, 64, 14, 28, 64"
        }
      ));
      
      speed_config_size_ = speed_config_vector_.size();
      
      // Subscriber
      this->joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&WhillJoy::ros_joy_callback_, this, std::placeholders::_1));

      // Client
      this->set_speed_profile_client_ = this->create_client<ros2_whill_interfaces::srv::SetSpeedProfile>("set_speed_profile_srv", rclcpp::QoS(5));
      this->set_power_client_ = this->create_client<ros2_whill_interfaces::srv::SetPower>("set_power_srv", rclcpp::QoS(5));

      if (this->set_speed_profile_client_->wait_for_service())
      {
        initial_speed_profiles_();
        set_speed_();
      }
    }

  private:
    //
    // ROS2 Objects
    //
    
    // Parameters
    long unsigned int frequency_;
    size_t speed_step_;
    double pressed_duration_;
    size_t speed_config_size_;

    // Parameters
    bool power_on_ = false;
    std::vector<ros2_whill_interfaces::srv::SetSpeedProfile::Request::SharedPtr> speed_profiles_;
    std::vector<std::vector<int8_t>> speed_config_vector_;

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // Client
    rclcpp::Client<ros2_whill_interfaces::srv::SetSpeedProfile>::SharedPtr set_speed_profile_client_;
    rclcpp::Client<ros2_whill_interfaces::srv::SetPower>::SharedPtr set_power_client_;
    //convert time
    // double nanosecToSec(const rcl_time_point_value_t nanoseconds);
    // double toSec(const rclcpp::Duration & duration);
    // double toSec(const rclcpp::Time & time);

    void ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy);
    void set_speed_();
    void set_power_();
    void initial_speed_profiles_();
    std::vector<std::vector<int8_t>> strToInt(const std::vector<std::string> &string_vector);
    bool check_speed_config(const std::vector<std::vector<int8_t>> &speed_config_vector);
    
};

void WhillJoy::ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if(joy->axes.size() < 8 || joy->buttons.size() < 11) return;

  // If button LB or RB is pressed
  if (joy->buttons[Buttons::LB] != joy->buttons[Buttons::RB])
  {
    if (joy->buttons[Buttons::LB] == 1)
    {
      speed_step_ --;
      set_speed_();
      RCLCPP_INFO(this->get_logger(), "Whill speed mode -1 !");
    }
    if (joy->buttons[Buttons::RB] == 1)
    {
      speed_step_ ++;
      set_speed_();
      RCLCPP_INFO(this->get_logger(), "Whill speed mode +1 !");
    }
  }

  // If button HOME is pressed
  if (joy->buttons[Buttons::HOME] == 1 && set_power_client_->service_is_ready())
  {
    set_power_();
  }

}

void WhillJoy::set_speed_()
{
  if (speed_step_ <= 0)
  {
    speed_step_ = 0;
    RCLCPP_WARN(this->get_logger(), "The speed of Whill can not be slower!!!");
  }
  else if (speed_step_ >= speed_config_size_ - 1)
  {
    speed_step_ = speed_config_size_ - 1;
    RCLCPP_WARN(this->get_logger(), "The speed of Whill can not be faster!!!");
  }
  
  auto req_future = this->set_speed_profile_client_->async_send_request(speed_profiles_[speed_step_]);
  
  RCLCPP_INFO(this->get_logger(), "Speed mode is changed to %ld", speed_step_);

  rclcpp::sleep_for(std::chrono::nanoseconds(sec_to_nano(pressed_duration_)));
}

void WhillJoy::initial_speed_profiles_()
{
  float l_scale, a_scale;
  if (!check_speed_config(speed_config_vector_) || speed_config_size_ == 0)
  {
    RCLCPP_WARN(this->get_logger(), "Bad speed configuration. Set it to default!");
    speed_config_vector_ = std::vector<std::vector<int8_t>>{
      {10, 16, 48, 10, 16, 40, 8, 56, 72},
      {15, 16, 64, 10, 16, 56, 10, 56, 72},
      {30, 16, 82, 20, 16, 64, 15, 56, 72},
      {45, 16, 90, 20, 24, 64, 18, 56, 72},
      {57, 16, 90, 20, 24, 64, 14, 28, 64},
    };
    speed_config_size_ = 5;
  }

  if (speed_step_ >= 0 && speed_step_ < speed_config_size_)
  {
    speed_profiles_.resize(speed_config_size_);
    RCLCPP_INFO(this->get_logger(), "Available Step: %ld", speed_config_size_);
  }
  else
  {
    speed_step_ = speed_config_size_ / 2;
    RCLCPP_WARN(this->get_logger(), "Bad Speed Step Configuration, and Set default one into the half of total steps.");
  }  

  for (size_t i = 0; i < speed_config_size_; i++)
  {
      speed_profiles_[i] = std::make_shared<ros2_whill_interfaces::srv::SetSpeedProfile::Request>();

      speed_profiles_[i]->fm1 = speed_config_vector_[i][0];
      speed_profiles_[i]->fa1 = speed_config_vector_[i][1];
      speed_profiles_[i]->fd1 = speed_config_vector_[i][2];
      speed_profiles_[i]->rm1 = speed_config_vector_[i][3];
      speed_profiles_[i]->ra1 = speed_config_vector_[i][4];
      speed_profiles_[i]->rd1 = speed_config_vector_[i][5];
      speed_profiles_[i]->tm1 = speed_config_vector_[i][6];
      speed_profiles_[i]->ta1 = speed_config_vector_[i][7];
      speed_profiles_[i]->td1 = speed_config_vector_[i][8];
      speed_profiles_[i]->s1 = 4;
      
      if (i == speed_step_)
      {
        // fm1とtm1は、0.1km/h単位のため、m/s単位とrad/sに変換する, 0.225は車軸の1/2である。
        l_scale = speed_profiles_[i]->fm1 * 0.1f * 1000 / 3600;
        a_scale = speed_profiles_[i]->tm1 * 0.1f * 1000 / 3600 / 0.225;
      }

  }

  RCLCPP_INFO(this->get_logger(), "Linear Velocity Maximum: %f, Angular velocity Maximum: %f", l_scale, a_scale);
}

void WhillJoy::set_power_()
{
  if (power_on_ == true)
  {
    auto req = std::make_shared<ros2_whill_interfaces::srv::SetPower::Request>();
    req->p0 = 0;
    set_power_client_->async_send_request(req);
    power_on_ = false;
    RCLCPP_INFO(get_logger(), "Set Whill Power to OFF!!!");
  }
  else
  {
    auto req = std::make_shared<ros2_whill_interfaces::srv::SetPower::Request>();
    req->p0 = 1;
    set_power_client_->async_send_request(req);
    power_on_ = true;
    RCLCPP_INFO(get_logger(), "Set Whill Power to ON!!!");
  }

  rclcpp::sleep_for(std::chrono::nanoseconds(sec_to_nano(pressed_duration_)));
}

std::vector<std::vector<int8_t>> WhillJoy::strToInt(const std::vector<std::string> &string_vector)
{
  std::vector<std::vector<int8_t>> result;

  for (auto &&string : string_vector)
  {
    std::vector<int8_t> vector;
    std::istringstream iss(string);
    std::string token;

    while (std::getline(iss, token, ','))
    {
      try{
          vector.push_back(std::stoi(token));
      } catch(const std::exception& e) {
          std::cerr << "Error converting string to int: " << e.what() << std::endl;
      }
    }
    result.push_back(vector);
  }

  return result;
}

bool WhillJoy::check_speed_config(const std::vector<std::vector<int8_t>> &speed_config_vector)
{
  for (auto &&config : speed_config_vector)
  {
    if (config.size() != size_t(CONFIG_SIZE))return false;

    // debug
    // printf("%d, %d, %d, %d, %d, %d, %d, %d, %d\n", 
    // config[0], config[1], config[2], config[3], config[4], config[5] ,config[6], config[7], config[8]);
  }
  return true;
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
    options.clock_type(RCL_SYSTEM_TIME);
    
    // プログラムの処理を終わらないようにするため、プロセスをスピンさせる、引数は：Nodeのオブジェクトのポインター
    rclcpp::spin(std::make_shared<WhillJoy>(options));

    // Nodeをシャットダウンする
    rclcpp::shutdown();

    return 0;
}