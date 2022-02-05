//
// Created by peterl328 on 2/4/22.
//

#ifndef TELEOP_JOYSTICK_H
#define TELEOP_JOYSTICK_H

#include <ros/ros.h>
#include <string>

#include <sensor_msgs/Joy.h>

class TeleopJoystick {
public:
    /// Creates an instance of TeleopJoystick
    /// \param nh The ROS node handler.
    TeleopJoystick(ros::NodeHandle *nh);

    /// Callback for converting sensor_msgs::Joy into cmd_vel and hexapod_msgs messages.
    /// \param legs_joints A reference to the LegJoints message.
    void joy_to_control_messages_callback(const sensor_msgs::Joy::ConstPtr &joy);
private:
    const std::string joy_topic_name_{"joy"};
    const std::string cmd_vel_topic_name_{"cmd_vel"};
    const std::string state_update_topic_name_{"state"};
    const std::string translate_rotate_command_topic_name_{"translate_rotate_command"};

    ros::Subscriber joy_subscriber_;
    ros::Publisher cmd_vel_publisher_;
    ros::Publisher state_update_publisher_;
    ros::Publisher translate_rotate_command_update_publisher_;

    bool is_turned_on{false};
    bool is_translate_mode_{false};
    float max_walking_speed_;
    float max_shift_distance_;
    float max_rotate_degree_;
};


#endif //TELEOP_JOYSTICK_H
