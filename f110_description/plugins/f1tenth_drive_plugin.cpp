#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <memory>
#include <string>
#include <cmath>

namespace gazebo
{
  class F1TenthDrivePlugin : public ModelPlugin
  {
  public:
    F1TenthDrivePlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
    {
      // Store the model pointer
      this->model_ = _model;

      // Initialize ROS node
      this->ros_node_ = gazebo_ros::Node::Get(_sdf);

      // Get parameters from SDF
      this->update_rate_ = 100.0;
      if (_sdf->HasElement("update_rate")) {
        this->update_rate_ = _sdf->Get<double>("update_rate");
      }

      // Get joint names
      std::string front_left_wheel_joint = "left_front_wheel_joint";
      std::string front_right_wheel_joint = "right_front_wheel_joint";
      std::string rear_left_wheel_joint = "left_rear_wheel_joint";
      std::string rear_right_wheel_joint = "right_rear_wheel_joint";
      std::string front_left_steering_joint = "left_steering_hinge_joint";
      std::string front_right_steering_joint = "right_steering_hinge_joint";

      if (_sdf->HasElement("front_left_wheel_joint"))
        front_left_wheel_joint = _sdf->Get<std::string>("front_left_wheel_joint");
      if (_sdf->HasElement("front_right_wheel_joint"))
        front_right_wheel_joint = _sdf->Get<std::string>("front_right_wheel_joint");
      if (_sdf->HasElement("rear_left_wheel_joint"))
        rear_left_wheel_joint = _sdf->Get<std::string>("rear_left_wheel_joint");
      if (_sdf->HasElement("rear_right_wheel_joint"))
        rear_right_wheel_joint = _sdf->Get<std::string>("rear_right_wheel_joint");
      if (_sdf->HasElement("front_left_steering_joint"))
        front_left_steering_joint = _sdf->Get<std::string>("front_left_steering_joint");
      if (_sdf->HasElement("front_right_steering_joint"))
        front_right_steering_joint = _sdf->Get<std::string>("front_right_steering_joint");

      // Get vehicle parameters
      this->wheelbase_ = 0.3302;
      this->track_width_ = 0.2413;
      this->wheel_radius_ = 0.0508;
      this->max_steer_angle_ = 0.4189;
      this->wheel_rotation_sign_ = 1.0;  // +1 keeps current sign, -1 flips

      if (_sdf->HasElement("wheelbase"))
        this->wheelbase_ = _sdf->Get<double>("wheelbase");
      if (_sdf->HasElement("track_width"))
        this->track_width_ = _sdf->Get<double>("track_width");
      if (_sdf->HasElement("wheel_radius"))
        this->wheel_radius_ = _sdf->Get<double>("wheel_radius");
      if (_sdf->HasElement("max_steer_angle"))
        this->max_steer_angle_ = _sdf->Get<double>("max_steer_angle");
      if (_sdf->HasElement("wheel_rotation_sign"))
        this->wheel_rotation_sign_ = _sdf->Get<double>("wheel_rotation_sign");

      // Get joints
      this->front_left_wheel_joint_ = this->model_->GetJoint(front_left_wheel_joint);
      this->front_right_wheel_joint_ = this->model_->GetJoint(front_right_wheel_joint);
      this->rear_left_wheel_joint_ = this->model_->GetJoint(rear_left_wheel_joint);
      this->rear_right_wheel_joint_ = this->model_->GetJoint(rear_right_wheel_joint);
      this->front_left_steering_joint_ = this->model_->GetJoint(front_left_steering_joint);
      this->front_right_steering_joint_ = this->model_->GetJoint(front_right_steering_joint);

      if (!this->front_left_wheel_joint_ || !this->front_right_wheel_joint_ ||
          !this->rear_left_wheel_joint_ || !this->rear_right_wheel_joint_ ||
          !this->front_left_steering_joint_ || !this->front_right_steering_joint_)
      {
        RCLCPP_ERROR(this->ros_node_->get_logger(), "Could not find all required joints!");
        return;
      }

      // Subscribe to /drive topic with compatible QoS
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
      qos.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Match topic pub default

      this->drive_sub_ = this->ros_node_->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
        "/drive", qos,
        std::bind(&F1TenthDrivePlugin::OnDriveCmd, this, std::placeholders::_1));

      // Listen to the update event
      this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&F1TenthDrivePlugin::OnUpdate, this));

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "F1TENTH Drive Plugin loaded! Listening on /drive topic");
      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "Vehicle params: wheelbase=%.3f, track=%.3f, wheel_radius=%.3f",
                  this->wheelbase_, this->track_width_, this->wheel_radius_);
      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "Control params: wheel_rotation_sign=%.1f",
                  this->wheel_rotation_sign_);
    }

    void OnDriveCmd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->target_speed_ = static_cast<double>(msg->drive.speed);
      double steering_angle = static_cast<double>(msg->drive.steering_angle);
      this->target_steering_angle_ = std::max(-this->max_steer_angle_,
                                               std::min(this->max_steer_angle_,
                                                       steering_angle));
      this->received_first_command_ = true;  // Mark that we've received a command

      // Debug log
      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "Received /drive: speed=%.2f m/s, steer=%.3f rad",
                  this->target_speed_, this->target_steering_angle_);
    }

    void OnUpdate()
    {
      std::lock_guard<std::mutex> lock(this->mutex_);

      // Don't do anything until we receive the first command
      if (!this->received_first_command_) {
        return;
      }

      // Calculate steering angles using Ackermann geometry
      double left_steer = 0.0;
      double right_steer = 0.0;
      double left_wheel_speed = 0.0;
      double right_wheel_speed = 0.0;

      if (std::abs(this->target_steering_angle_) < 0.001)
      {
        // Straight line
        left_steer = 0.0;
        right_steer = 0.0;
        left_wheel_speed = (this->target_speed_ / this->wheel_radius_);
        right_wheel_speed = (this->target_speed_ / this->wheel_radius_);
      }
      else
      {
        // Calculate turning radius at center of rear axle
        double turning_radius = this->wheelbase_ / std::tan(std::abs(this->target_steering_angle_));

        // Calculate inner and outer steering angles (Ackermann)
        if (this->target_steering_angle_ > 0)  // Left turn
        {
          left_steer = std::atan(this->wheelbase_ / (turning_radius - this->track_width_ / 2.0));
          right_steer = std::atan(this->wheelbase_ / (turning_radius + this->track_width_ / 2.0));
        }
        else  // Right turn
        {
          left_steer = -std::atan(this->wheelbase_ / (turning_radius + this->track_width_ / 2.0));
          right_steer = -std::atan(this->wheelbase_ / (turning_radius - this->track_width_ / 2.0));
        }

        // Calculate angular velocity
        double angular_vel = this->target_speed_ / turning_radius;

        // Calculate wheel velocities considering differential drive
        double left_linear_vel = this->target_speed_ - (this->track_width_ / 2.0) * angular_vel;
        double right_linear_vel = this->target_speed_ + (this->track_width_ / 2.0) * angular_vel;

        // Convert to wheel angular velocity (desired, in vehicle-forward convention)
        left_wheel_speed = (left_linear_vel / this->wheel_radius_);
        right_wheel_speed = (right_linear_vel / this->wheel_radius_);
      }
      // For older Gazebo, use PID with high gains
      double current_left_steer = this->front_left_steering_joint_->Position(0);
      double current_right_steer = this->front_right_steering_joint_->Position(0);

      double steer_kp = 200.0;  // Steering position gain
      double left_steer_error = left_steer - current_left_steer;
      double right_steer_error = right_steer - current_right_steer;

      this->front_left_steering_joint_->SetForce(0, steer_kp * left_steer_error);
      this->front_right_steering_joint_->SetForce(0, steer_kp * right_steer_error);


      // Apply wheel velocities - use PD control with limited torque
      // Get current velocities (raw, in joint axis convention)
      double current_left_vel_raw = this->rear_left_wheel_joint_->GetVelocity(0);
      double current_right_vel_raw = this->rear_right_wheel_joint_->GetVelocity(0);

      // Correct measured sign to vehicle-forward convention
      double current_left_vel = this->wheel_rotation_sign_ * current_left_vel_raw;
      double current_right_vel = this->wheel_rotation_sign_ * current_right_vel_raw;

      // Calculate velocity error
      double left_vel_error = left_wheel_speed - current_left_vel;
      double right_vel_error = right_wheel_speed - current_right_vel;

      // PD control gains
      double wheel_kp = 5.0;   // Proportional gain (reduced for smoother control)
      double wheel_kd = 0.5;   // Derivative gain (for damping)

      // Calculate derivative (approximate with error difference)
      double left_vel_deriv = left_vel_error - this->prev_left_error_;
      double right_vel_deriv = right_vel_error - this->prev_right_error_;

      this->prev_left_error_ = left_vel_error;
      this->prev_right_error_ = right_vel_error;

      // Apply PD control
      double left_force_pd = wheel_kp * left_vel_error + wheel_kd * left_vel_deriv;
      double right_force_pd = wheel_kp * right_vel_error + wheel_kd * right_vel_deriv;

      // Limit maximum torque
      double max_wheel_force = 50.0;  // Reduced max torque
      double left_force = std::max(-max_wheel_force, std::min(max_wheel_force, left_force_pd));
      double right_force = std::max(-max_wheel_force, std::min(max_wheel_force, right_force_pd));

      // Map control output back to joint axis convention
      left_force *= this->wheel_rotation_sign_;
      right_force *= this->wheel_rotation_sign_;

      this->rear_left_wheel_joint_->SetForce(0, left_force);
      this->rear_right_wheel_joint_->SetForce(0, right_force);

      // No drive torque on front wheels (RWD only)
      this->front_left_wheel_joint_->SetForce(0, 0.0);
      this->front_right_wheel_joint_->SetForce(0, 0.0);

      // Debug log every 100 iterations
    static int counter = 0;
    if (counter++ % 100 == 0) {
        // Get all wheel velocities
        double fl_vel = this->front_left_wheel_joint_->GetVelocity(0);
        double fr_vel = this->front_right_wheel_joint_->GetVelocity(0);
        double rl_vel = current_left_vel_raw;
        double rr_vel = current_right_vel_raw;
        
        RCLCPP_INFO(this->ros_node_->get_logger(),
          "Wheel Velocities (rad/s) - FL:%.2f FR:%.2f RL:%.2f RR:%.2f | "
          "Target: L=%.2f R=%.2f | Forces: L=%.2f R=%.2f | "
          "Steer: target_L=%.3f current_L=%.3f target_R=%.3f current_R=%.3f",
          fl_vel, fr_vel, rl_vel, rr_vel,
          left_wheel_speed, right_wheel_speed,
          left_force, right_force,
          left_steer, current_left_steer, right_steer, current_right_steer);
      }
    }

  private:
    // Model pointer
    physics::ModelPtr model_;

    // ROS node
    gazebo_ros::Node::SharedPtr ros_node_;

    // Joints
    physics::JointPtr front_left_wheel_joint_;
    physics::JointPtr front_right_wheel_joint_;
    physics::JointPtr rear_left_wheel_joint_;
    physics::JointPtr rear_right_wheel_joint_;
    physics::JointPtr front_left_steering_joint_;
    physics::JointPtr front_right_steering_joint_;

    // Parameters
    double wheelbase_;
    double track_width_;
    double wheel_radius_;
    double max_steer_angle_;
    double update_rate_;
    double wheel_rotation_sign_;

    // Target values
    double target_speed_ = 0.0;
    double target_steering_angle_ = 0.0;

    // Control flags
    bool received_first_command_ = false;

    // PD control state
    double prev_left_error_ = 0.0;
    double prev_right_error_ = 0.0;

    // Mutex
    std::mutex mutex_;

    // Subscriber
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;

    // Update connection
    event::ConnectionPtr update_connection_;
  };

  GZ_REGISTER_MODEL_PLUGIN(F1TenthDrivePlugin)
}
