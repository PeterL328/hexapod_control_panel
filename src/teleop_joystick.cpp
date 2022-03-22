//
// Created by peterl328 on 2/4/22.
//

#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <hexapod_msgs/Pose.h>

#include "teleop_joystick.h"

TeleopJoystick::TeleopJoystick(ros::NodeHandle *nh) {
    // Load params
    nh->param("max_walking_speed", max_walking_speed_, 0.2f);
    nh->param("max_shift_distance", max_shift_distance_, 0.04f);
    nh->param("max_rotate_degree", max_rotate_degree_, 0.2f);
    nh->param("max_stationary_rotate_degree", max_stationary_rotate_degree_, 0.25f);

    joy_subscriber_ = nh->subscribe(joy_topic_name_, 1, &TeleopJoystick::joy_to_control_messages_callback, this);
    cmd_vel_publisher_ = nh->advertise<geometry_msgs::Twist>(cmd_vel_topic_name_, 1);
    state_update_publisher_ = nh->advertise<std_msgs::String>(state_update_topic_name_, 1);
    translate_rotate_command_update_publisher_ = nh->advertise<hexapod_msgs::Pose>(translate_rotate_command_topic_name_, 1);
}

void TeleopJoystick::joy_to_control_messages_callback(const sensor_msgs::Joy::ConstPtr &joy) {
    if (joy->buttons[7] || joy->buttons[3]) {
        std_msgs::String state_message;
        if (joy->buttons[7]) {
            if (is_turned_on) {
                state_message.data = "Off";
            } else {
                state_message.data = "Normal";
            }
            is_turned_on = !is_turned_on;
        } else {
            if (is_translate_mode_) {
                state_message.data = "Normal";
            } else {
                state_message.data = "TranslateRotate";
            }
            is_translate_mode_ = !is_translate_mode_;
        }
        state_update_publisher_.publish(state_message);
    }

    // Read joy axis values
    float joy0_x = joy->axes[0];
    float joy0_y = joy->axes[1];
    float joy1_x = joy->axes[3];
    float joy1_y = joy->axes[4];

    if (!is_translate_mode_) {
        geometry_msgs::Twist cmd_vel_message;
        cmd_vel_message.linear.x = -joy0_x * max_walking_speed_;
        cmd_vel_message.linear.y = joy0_y * max_walking_speed_;
        cmd_vel_message.angular.x = -joy1_x * max_rotate_degree_;
        cmd_vel_publisher_.publish(cmd_vel_message);
    } else {
        hexapod_msgs::Pose pose_message;
        pose_message.position.x = -joy0_x * max_shift_distance_;
        pose_message.position.y = joy0_y * max_shift_distance_;

        pose_message.orientation.yaw = -joy1_x * max_stationary_rotate_degree_;
        pose_message.orientation.pitch = -joy1_y * max_stationary_rotate_degree_;
        translate_rotate_command_update_publisher_.publish(pose_message);
    }
}

int main(int argc, char **argv)
{
    ROS_INFO("Teleop-joystick started.");
    const std::string node_name = "teleop_joystick";
    ros::init(argc, argv, node_name);
    ros::NodeHandle n("~");

    TeleopJoystick teleop_joystick{&n};

    ros::spin();
    return 0;
}