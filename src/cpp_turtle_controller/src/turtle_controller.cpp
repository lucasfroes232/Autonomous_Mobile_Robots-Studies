// turtle_controller.cpp — ROS 2 (C++)
// Equivalente ao TurtleController em Python

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <random>
#include <vector>

static constexpr int   NUM_WAYPOINT    = 4;
static constexpr float DIST_TOLERANCE  = 0.3f;

class TurtleController : public rclcpp::Node
{
public:
  TurtleController()
  : Node("turtle_controller")
  {
    // Publisher: envia comandos de velocidade para a tartaruga
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10);

    // Subscriber: recebe a posição da tartaruga
    subscription_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtleController::pose_callback, this, std::placeholders::_1));

    // Timer para publicar comandos periodicamente
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TurtleController::control_loop, this));

    // Inicializa pose como nullptr
    pose_ = nullptr;

    // Cria waypoints e reseta índice
    create_list_waypoint();
    current_waypoint_ = 0;
    done_ = false;
  }

private:
  // ─── Membros ────────────────────────────────────────────────────────────────
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     publisher_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr       subscription_;
  rclcpp::TimerBase::SharedPtr                                timer_;
  turtlesim::msg::Pose::SharedPtr                             pose_;

  std::vector<std::pair<float, float>>  waypoint_;
  int                                   current_waypoint_;
  bool                                  done_;

  // ─── Funções ────────────────────────────────────────────────────────────────
  void create_list_waypoint()
  {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(1.0f, 10.0f);

    waypoint_.clear();
    for (int i = 0; i < NUM_WAYPOINT; ++i) {
      waypoint_.emplace_back(dist(gen), dist(gen));
    }

    RCLCPP_INFO(this->get_logger(), "Defined waypoints:");
    for (int i = 0; i < static_cast<int>(waypoint_.size()); ++i) {
      RCLCPP_INFO(this->get_logger(),
        "  WP[%d]: x=%.2f, y=%.2f", i, waypoint_[i].first, waypoint_[i].second);
    }
  }

  void pose_callback(const turtlesim::msg::Pose::SharedPtr msg)
  {
    pose_ = msg;
  }

  void control_loop()
  {
    if (!pose_ || done_) return;

    if (current_waypoint_ < static_cast<int>(waypoint_.size())) {
      float dx_goal = waypoint_[current_waypoint_].first;
      float dy_goal = waypoint_[current_waypoint_].second;

      float dx = dx_goal - pose_->x;
      float dy = dy_goal - pose_->y;
      float dist_error = std::sqrt(dx * dx + dy * dy);

      if (dist_error < DIST_TOLERANCE) {
        RCLCPP_INFO(this->get_logger(),
          "Objective (%.2f, %.2f) achieved", dx_goal, dy_goal);
        current_waypoint_++;
        return;
      }

      auto msg = geometry_msgs::msg::Twist();
      float theta_goal  = std::atan2(dy, dx);
      float theta_error = theta_goal - pose_->theta;

      if (std::abs(theta_error) > 0.1f) {
        msg.angular.z = 1.0f * theta_error;
        msg.linear.x  = 0.0f;
      } else {
        msg.angular.z = 0.0f;
        msg.linear.x  = 1.5f * dist_error;
      }

      publisher_->publish(msg);

    } else {
      done_ = true;
      publisher_->publish(geometry_msgs::msg::Twist()); // Para a tartaruga
      RCLCPP_INFO(this->get_logger(), "Achieved all points!");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TurtleController>());
  rclcpp::shutdown();
  return 0;
}