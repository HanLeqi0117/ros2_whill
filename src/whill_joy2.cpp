#include <iostream>
#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ros2_whill_interfaces/msg/whill_speed_profile.hpp"
#include "ros2_whill_interfaces/srv/set_speed_profile.hpp"

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
    WhillJoy(const rclcpp::NodeOptions &options) : Node("whill_joy2", options)
    {
      this->speed_step_ = this->declare_parameter("speed_step", 0);
      this->use_joycon_ = this->declare_parameter("use_joycon", true);
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

      // Initialize Time Instance
      joy_sub_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
      cmd_sub_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME); 
      pressed_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
      
      zero_twist_published_ = false;
      stop_pressed_ = false;
      joy_.axes.resize(8);
      zero_twist_.axes.resize(8);
      speed_profiles_.resize(6);
      joy_.axes[Axes::LEFT_STICK_L_R] = 0;
      joy_.axes[Axes::LEFT_STICK_U_D] = 0;
      zero_twist_.axes[Axes::LEFT_STICK_L_R] = 0;
      zero_twist_.axes[Axes::LEFT_STICK_U_D] = 0;
      speed_config_size_ = speed_config_vector_.size();

      // Publisher
      this->joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("controller/joy", 100);
      
      // Subscriber
      this->joy_state_subscriber_ = create_subscription<sensor_msgs::msg::Joy>("joy_state", 10, std::bind(&WhillJoy::ros_joy_callback_, this, std::placeholders::_1));
      this->cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("controller/cmd_vel", 10, std::bind(&WhillJoy::ros_cmd_vel_callback_, this, std::placeholders::_1));

      // Client
      this->set_speed_profile_client_ = this->create_client<ros2_whill_interfaces::srv::SetSpeedProfile>("set_speed_profile_srv", rclcpp::QoS(5));

      // WallTimer
      joy_pub_timer_ = this->create_wall_timer(
        1.0s / this->frequency_,
        std::bind(&WhillJoy::main_process_, this)
      ); 

      if (this->set_speed_profile_client_->wait_for_service())
      {
        initial_speed_profiles_();
        set_speed_(speed_step_);
        can_set_speed_ = true;
      }
    }

  private:
    //
    // ROS2 Objects
    //

    // Time
    rclcpp::Time joy_sub_time_, cmd_sub_time_;
    rclcpp::Time pressed_time_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr joy_pub_timer_;

    // Duraton
    // rclcpp::Duration joy_sub_duration_;
    rclcpp::Duration joy_sub_duration_ = rclcpp::Duration::from_seconds(0.0);

    // Parameters
    long unsigned int frequency_;
    size_t speed_step_;
    bool use_joycon_;
    double a_scale_;
    double l_scale_;
    double pressed_duration_;
    size_t speed_config_size_;

    // Parameters
    bool zero_twist_published_;
    bool stop_pressed_;
    bool can_set_speed_ = false;
    bool lb_triggered_ = false;
    bool rb_triggered_ = false;
    sensor_msgs::msg::Joy joy_, zero_twist_;
    std::vector<ros2_whill_interfaces::msg::WhillSpeedProfile> speed_profiles_;
    std::vector<std::vector<int8_t>> speed_config_vector_;
    int speed_profiles_count_ = 0;

    // Publisherを宣言する
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;

    // Subscriberのを宣言する
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_state_subscriber_;

    // Client
    rclcpp::Client<ros2_whill_interfaces::srv::SetSpeedProfile>::SharedPtr set_speed_profile_client_;

    //convert time
    // double nanosecToSec(const rcl_time_point_value_t nanoseconds);
    // double toSec(const rclcpp::Duration & duration);
    // double toSec(const rclcpp::Time & time);

    void main_process_();
    void ros_cmd_vel_callback_(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel);
    void ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy_state);
    void set_speed_(size_t now_config);
    void initial_speed_profiles_();
    std::vector<std::vector<int8_t>> strToInt(const std::vector<std::string> &string_vector);
    bool check_speed_config(const std::vector<std::vector<int8_t>> &speed_config_vector);
    
};

// double WhillJoy::nanosecToSec(const rcl_time_point_value_t nanoseconds)
// {
//   return static_cast<double>(nanoseconds) * 1e-9;
// }

// double WhillJoy::toSec(const rclcpp::Duration & duration)
// {
//   return WhillJoy::nanosecToSec(duration.nanoseconds());
// }

// double WhillJoy::toSec(const rclcpp::Time & time)
// {
//   return WhillJoy::nanosecToSec(time.nanoseconds());
// }

void WhillJoy::ros_joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy_state)
{

  joy_sub_time_ = this->now();
  rclcpp::Duration pressed_duration(this->now() - pressed_time_);

  if(joy_state->axes.size() < 8 || joy_state->buttons.size() < 11) return;

  if (joy_state->buttons[Buttons::LB] == 1)
  {
    if (pressed_duration.seconds() > pressed_duration_)
    {
      this->pressed_time_ = this->now();
      lb_triggered_ = true;
    }
  }
  if (joy_state->buttons[Buttons::RB] == 1)
  {
    if (pressed_duration.seconds() > pressed_duration_)
    {
      this->pressed_time_ = this->now();
      rb_triggered_ = true;
    }
  }

  if (joy_state->buttons[Buttons::LB] == 1 && joy_state->buttons[Buttons::RB] == 1)
  {  
    rb_triggered_ = false;
    lb_triggered_ = false;
  }
  
  if(joy_state->axes[Axes::LT] >= 0 && joy_state->axes[Axes::RT] >= 0)
  {
    if(joy_state->buttons[Buttons::X] <= 0)
    {
      joy_.axes[Axes::LEFT_STICK_L_R] = joy_state->axes[Axes::LEFT_STICK_L_R];
      joy_.axes[Axes::LEFT_STICK_U_D] = joy_state->axes[Axes::LEFT_STICK_U_D];    
    }
    else if(joy_state->buttons[Buttons::X] > 0)
    {
      joy_.axes[Axes::LEFT_STICK_L_R] = 0.0;
      joy_.axes[Axes::LEFT_STICK_U_D] = joy_state->axes[Axes::LEFT_STICK_U_D];
    }
  }
  else if(joy_state->axes[Axes::RT] < 0)
  {
    joy_.axes[Axes::LEFT_STICK_L_R] = -1.0;
    joy_.axes[Axes::LEFT_STICK_U_D] = 0;
  }
  else if(joy_state->axes[Axes::LT] < 0)
  {
    joy_.axes[Axes::LEFT_STICK_L_R] = 1.0;
    joy_.axes[Axes::LEFT_STICK_U_D] = 0;
  }
  stop_pressed_ = joy_state->buttons[Buttons::A];
}

void WhillJoy::ros_cmd_vel_callback_(geometry_msgs::msg::Twist::ConstSharedPtr cmd_vel)
{
  cmd_sub_time_ = this->now();
  if (lb_triggered_ || rb_triggered_) return;

  if(cmd_vel->angular.z < a_scale_ && cmd_vel->angular.z > -1.0 * a_scale_)
    joy_.axes[Axes::LEFT_STICK_L_R] = cmd_vel->angular.z / a_scale_;
  else if(cmd_vel->angular.z > a_scale_)
    joy_.axes[Axes::LEFT_STICK_L_R] = 1.0;
  else
    joy_.axes[Axes::LEFT_STICK_L_R] = -1.0;
  
  if(cmd_vel->linear.x < l_scale_ && cmd_vel->linear.x > -1.0 * l_scale_)
    joy_.axes[Axes::LEFT_STICK_U_D] = cmd_vel->linear.x / l_scale_;
  else if(cmd_vel->linear.x > l_scale_)
    joy_.axes[Axes::LEFT_STICK_U_D] = 1.0;
  else
    joy_.axes[Axes::LEFT_STICK_U_D] = -1.0;
}

void WhillJoy::main_process_()
{
  if(!use_joycon_)
  {
    joy_sub_duration_ = this->now() - cmd_sub_time_;
    // RCLCPP_INFO(this->get_logger(), "Prepared to send Joy!!! Time is %.2fs = %ldns - %ldns", joy_sub_duration_.seconds(), this->now().nanoseconds(), cmd_sub_time_.nanoseconds());
    if (joy_sub_duration_.seconds() < 0.2) this->joy_publisher_->publish(joy_);
  }
  else{
    joy_sub_duration_ = this->now() - joy_sub_time_;
    if (!stop_pressed_ && joy_sub_duration_.seconds() < 0.2)
    {
      // 速度指令をそのままpublishする
      this->joy_publisher_->publish(joy_);
      zero_twist_published_ = false;
    }
    else{
      this->joy_publisher_->publish(zero_twist_);
      zero_twist_published_ = true;
      // std::cout << "vel zero" << std::endl;
    }

    if (can_set_speed_)
    {
      if (this->set_speed_profile_client_->service_is_ready())
      {
        if (lb_triggered_)
        {
          speed_step_--;
          set_speed_(speed_step_);
          RCLCPP_INFO(this->get_logger(), "Whill speed mode -1 !");
          lb_triggered_ = false;
        }
        if (rb_triggered_)
        {
          speed_step_++;
          set_speed_(speed_step_);
          RCLCPP_INFO(this->get_logger(), "Whill speed mode +1 !");
          rb_triggered_ = false;
        }
      }
    }

  }
}

void WhillJoy::set_speed_(size_t now_config)
{

  if (now_config < 0)
  {
    speed_step_ ++;
    RCLCPP_WARN(this->get_logger(), "The speed of Whill can not be slower!!!");
    return;
  }
  else if (now_config > speed_config_size_ - 1)
  {
    speed_step_ --;
    RCLCPP_WARN(this->get_logger(), "The speed of Whill can not be faster!!!");
    return;
  }

  auto req = std::make_shared<ros2_whill_interfaces::srv::SetSpeedProfile::Request>();
  
  req->s1 = 4;
  req->fm1 = speed_profiles_[now_config].fm1;
  req->fa1 = speed_profiles_[now_config].fa1;
  req->fd1 = speed_profiles_[now_config].fd1;
  req->rm1 = speed_profiles_[now_config].rm1;
  req->ra1 = speed_profiles_[now_config].ra1;
  req->rd1 = speed_profiles_[now_config].rd1;
  req->tm1 = speed_profiles_[now_config].tm1;
  req->ta1 = speed_profiles_[now_config].ta1;
  req->td1 = speed_profiles_[now_config].td1;
  auto req_future = this->set_speed_profile_client_->async_send_request(req);
  
  RCLCPP_INFO(this->get_logger(), "Speed mode is changed to %ld", now_config);
  
}

void WhillJoy::initial_speed_profiles_()
{
  if (check_speed_config(speed_config_vector_) || speed_config_size_ == 0)
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
    RCLCPP_INFO(this->get_logger(), "Available Step: %ld", speed_config_size_);
  else
  {
    speed_step_ = speed_config_size_ / 2;
    RCLCPP_WARN(this->get_logger(), "Bad Speed Step Configuration, and Set default one into the half of total steps.");
  }  

  for (size_t i = 0; i < speed_config_size_; i++)
  {
      speed_profiles_[i].fm1 = speed_config_vector_[i][0];
      speed_profiles_[i].fa1 = speed_config_vector_[i][1];
      speed_profiles_[i].fd1 = speed_config_vector_[i][2];
      speed_profiles_[i].rm1 = speed_config_vector_[i][3];
      speed_profiles_[i].ra1 = speed_config_vector_[i][4];
      speed_profiles_[i].rd1 = speed_config_vector_[i][5];
      speed_profiles_[i].tm1 = speed_config_vector_[i][6];
      speed_profiles_[i].ta1 = speed_config_vector_[i][7];
      speed_profiles_[i].td1 = speed_config_vector_[i][8];
      
      if (i == speed_step_)
      {
        // fm1とtm1は、0.1km/h単位のため、m/s単位とrad/sに変換する, 0.225は車軸の1/2である。
        l_scale_ = speed_profiles_[i].fm1 * 0.1 * 1000 / 3600;
        a_scale_ = speed_profiles_[i].tm1 * 0.1 * 1000 / 3600 / 0.225;
      }

  }

  RCLCPP_INFO(this->get_logger(), "Linear Velocity Maximum: %f, Angular velocity Maximum: %f", l_scale_, a_scale_);

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