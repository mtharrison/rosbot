#include <functional>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "motor/config.h"
#include "motor/encoder.h"
#include "motor/odometry.h"
#include "motor/kinematics.h"
#include "motor/pid.h"
#include "motor/motor.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

Encoder *encoder_fl;
Encoder *encoder_fr;

int fl_dir;
int fr_dir;

void encoder1_callback(void) { 
  if(fl_dir == 1) {
    encoder_fl->ticks_ += 1;
  }
  if(fl_dir == -1) {
    encoder_fl->ticks_ -= 1;
  }
}
void encoder2_callback(void) { 
  if(fr_dir == 1) {
    encoder_fr->ticks_ += 1;
  }
  if(fr_dir == -1) {
    encoder_fr->ticks_ -= 1;
  }
}

#define GPIO_11 0
#define GPIO_13 2
#define GPIO_15 3
#define GPIO_16 4
#define GPIO_18 5
#define GPIO_19 12
#define GPIO_21 13
#define GPIO_22 6
#define GPIO_23 14
#define GPIO_24 10
#define GPIO_26 11
#define GPIO_32 26

Motor *motor1;
Motor *motor2;
Motor *motor3;
Motor *motor4;

unsigned long prev_cmd_time = 0;

class MotorFirmware : public rclcpp::Node
{
  public:
    MotorFirmware() : Node("motor_firmware")
    {
      this->declare_parameter("pid_kp", 1.0);
      this->declare_parameter("pid_ki", 0.0);
      this->declare_parameter("pid_kd", 0.0);
      this->declare_parameter("ang_v_multipler", 6.0);
      this->declare_parameter("lin_x_multipler", 6.0);

      this->get_parameter("pid_kp", kp_);
      this->get_parameter("pid_ki", ki_);
      this->get_parameter("pid_kd", kd_);
      this->get_parameter("ang_v_multipler", ang_v_multipler_);
      this->get_parameter("lin_x_multipler", lin_x_multipler_);

      printf("KP=%f | KI=%f | KD=%f | ANGV=%f\n", kp_, ki_, kd_, ang_v_multipler_);

      motor1_pid = new PID(PWM_MIN, PWM_MAX, kp_, ki_, kd_);
      motor2_pid = new PID(PWM_MIN, PWM_MAX, kp_, ki_, kd_);
      motor3_pid = new PID(PWM_MIN, PWM_MAX, kp_, ki_, kd_);
      motor4_pid = new PID(PWM_MIN, PWM_MAX, kp_, ki_, kd_);

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/unfiltered", 10);

      twist_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 
        10, 
        std::bind(&MotorFirmware::twist_callback, this, _1)
      );

      control_timer_ = this->create_wall_timer(
        20ms, std::bind(&MotorFirmware::control_callback, this)
      );

      prev_odom_update_ = 0;

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      kinematics_ = new Kinematics(
        Kinematics::SKID_STEER, 
        MOTOR_MAX_RPM, 
        MAX_RPM_RATIO, 
        MOTOR_OPERATING_VOLTAGE, 
        MOTOR_POWER_MAX_VOLTAGE, 
        WHEEL_DIAMETER, 
        LR_WHEELS_DISTANCE
      );

      odometry_ = new Odometry();
    }

  private:
    void twist_callback(const geometry_msgs::msg::Twist &msg)
    {
      twist_ = msg;
      twist_.linear.x = (twist_.linear.x);
      twist_.angular.z = (twist_.angular.z);
      prev_cmd_time = millis();
    }

    void moveBase()
    {
      if(((millis() - prev_cmd_time) >= 200)) 
      {
          twist_.linear.x = 0.0;
          twist_.linear.y = 0.0;
          twist_.angular.z = 0.0;
      }

      // Calculate desired rpms using PID controller
      Kinematics::rpm req_rpm = kinematics_->getRPM(
        twist_.linear.x, 
        twist_.linear.y, 
        twist_.angular.z
      );

      // Get current wheel rpm and odom velocities
      float fl_rpm = encoder_fl->getRPM();
      float fr_rpm = encoder_fr->getRPM();

      // Set RPMs using PID controller
      float target_pwm1 = motor1_pid->compute(req_rpm.motor1, fl_rpm);
      float target_pwm2 = motor2_pid->compute(req_rpm.motor2, fr_rpm);
      float target_pwm3 = motor3_pid->compute(req_rpm.motor3, fl_rpm);
      float target_pwm4 = motor4_pid->compute(req_rpm.motor4, fr_rpm);

      if (target_pwm1 < 0) {
        fl_dir = -1;
      }

      if (target_pwm1 == 0) {
        fl_dir = 0;
      }

      if (target_pwm1 > 0) {
        fl_dir = 1;
      }

      if (target_pwm2 < 0) {
        fr_dir = -1;
      }

      if (target_pwm2 == 0) {
        fr_dir = 0;
      }

      if (target_pwm2 > 0) {
        fr_dir = 1;
      }

      motor1->spin((int) target_pwm1);
      motor2->spin((int) target_pwm2);
      motor3->spin((int) target_pwm1);
      motor4->spin((int) target_pwm2);

      Kinematics::velocities current_vel = kinematics_->getVelocities(
          fl_rpm, 
          fr_rpm, 
          fl_rpm, 
          fr_rpm
      );

      printf("Current Vel = %f %f %f\n", current_vel.linear_x, current_vel.linear_y, current_vel.angular_z);
      printf("Twist Vel = %f %f %f\n", twist_.linear.x, twist_.linear.y, twist_.angular.z);
      printf("Motor 1 req_rpm = %f | actual_rpm = %f | pwm = %f/%d\n",  req_rpm.motor1, fl_rpm, target_pwm1, (int) target_pwm1);
      printf("Motor 2 req_rpm = %f | actual_rpm = %f | pwm = %f/%d\n", req_rpm.motor2, fr_rpm, target_pwm2, (int) target_pwm2);
      printf("Motor 3 req_rpm = %f | actual_rpm = %f | pwm = %f/%d\n", req_rpm.motor3, fl_rpm, target_pwm3, (int) target_pwm3);
      printf("Motor 4 req_rpm = %f | actual_rpm = %f | pwm = %f/%d\n\n\n", req_rpm.motor4, fr_rpm, target_pwm4, (int) target_pwm4);

      // update odometry
      unsigned long now = millis();
      float vel_dt = (now - prev_odom_update_) / 1000.0;
      prev_odom_update_ = now;
      odometry_->update(
          vel_dt, 
          current_vel.linear_x, 
          current_vel.linear_y, 
          current_vel.angular_z
      );
    }

    void publishData()
    {
      odom_msg_ = odometry_->getData();
      odom_msg_.header.stamp = this->get_clock()->now();
      odom_publisher_->publish(odom_msg_);
    }

    void control_callback()
    {
      moveBase();
      publishData();
    }

    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;
    unsigned long prev_odom_update_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    nav_msgs::msg::Odometry odom_msg_;
    geometry_msgs::msg::Twist twist_;

    Kinematics *kinematics_;
    Odometry *odometry_;
    PID *motor1_pid;
    PID *motor2_pid;
    PID *motor3_pid;
    PID *motor4_pid;

    float kp_;
    float ki_;
    float kd_;

    float ang_v_multipler_;
    float lin_x_multipler_;
};

int main(int argc, char* argv[])
{
  if (wiringPiSetup() < 0) {
      fprintf(stderr, "Unable to setup wiringPi: %s\n", strerror(errno));
      return 1;
  }

  fl_dir = 0;
  fr_dir = 0;

  encoder_fl = new Encoder(ENCODER_FL_PIN, &encoder1_callback);
  encoder_fr = new Encoder(ENCODER_FR_PIN, &encoder2_callback);

  motor1 = new Motor(GPIO_32, GPIO_26, GPIO_24);
  motor2 = new Motor(GPIO_22, GPIO_18, GPIO_16);
  motor3 = new Motor(GPIO_19, GPIO_21, GPIO_23);
  motor4 = new Motor(GPIO_11, GPIO_15, GPIO_13);
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorFirmware>());
  rclcpp::shutdown();    

  return 0;
}
