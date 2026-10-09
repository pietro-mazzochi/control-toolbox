// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "system_ident/joint_data_logger.hpp"

JointDataLogger::JointDataLogger() : Node("data_logger")
{
    this->declare_parameter<std::string>("output_file", "~/output.csv");
    output_file_name_ = this->get_parameter("output_file").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Config of 'output_file': " << output_file_name_);

    this->declare_parameter<int>("timer_frequency", 1);
    timer_frequency = this->get_parameter("timer_frequency").as_int();
    RCLCPP_INFO_STREAM(this->get_logger(), "Config of 'timer_frequency': " << timer_frequency);

    this->declare_parameter<std::string>("left_wheel_cmd_topic", "/left_command");
    std::string left_wheel_cmd_topic_ = this->get_parameter("left_wheel_cmd_topic").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Left wheel command topic: " << left_wheel_cmd_topic_);

    this->declare_parameter<std::string>("right_wheel_cmd_topic", "/right_command");
    std::string right_wheel_cmd_topic_ = this->get_parameter("right_wheel_cmd_topic").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Right wheel command topic: " << right_wheel_cmd_topic_);

    this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    std::string joint_states_topic = this->get_parameter("joint_states_topic").as_string();
    RCLCPP_INFO_STREAM(this->get_logger(), "Joint states topic: " << joint_states_topic);

    left_wheel_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(left_wheel_cmd_topic_, 10, std::bind(&JointDataLogger::left_joint_effort_callback, this, std::placeholders::_1));
    right_wheel_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(right_wheel_cmd_topic_, 10, std::bind(&JointDataLogger::right_joint_effort_callback, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_states_topic, 10, std::bind(&JointDataLogger::joint_states_callback, this, std::placeholders::_1));

    output_file_.open(output_file_name_.c_str(), std::ios::out | std::ios::app);
    timebase_ms_ = 0;

    // using namespace std::chrono_literals;
    // timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::milliseconds(1000 / timer_frequency), std::bind(&JointDataLogger::timer_callback, this));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / timer_frequency), std::bind(&JointDataLogger::timer_callback, this));

    // initial_time_ = this->get_clock()->now();

    joint_data_.name.resize(2);
    joint_data_.effort.resize(2);
    joint_data_.position.resize(2);
    joint_data_.velocity.resize(2);
    joint_data_.name[0] = "Left joint";
    joint_data_.name[1] = "Right joint";

    std::stringstream data_stream_;
    data_stream_ << "%%__time, header_timestamp, "
                 << "left_wheel_effort, left_wheel_position, left_wheel_velocity,"
                 << "right_wheel_effort, right_wheel_position, right_wheel_velocity";
    output_file_ << data_stream_.str() << std::endl;
    data_stream_.str().clear();
}

JointDataLogger::~JointDataLogger()
{
    output_file_.close();
}

void JointDataLogger::left_joint_effort_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    joint_data_.effort[0] = msg->data[0];
}

void JointDataLogger::right_joint_effort_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    joint_data_.effort[1] = msg->data[0];
}

void JointDataLogger::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    joint_data_.header = msg->header;
    joint_data_.velocity[0] = msg->velocity[0];
    joint_data_.velocity[1] = msg->velocity[1];

    joint_data_.position[0] = msg->position[0];
    joint_data_.position[1] = msg->position[1];

    // joint_data_.effort[0] = msg->effort[0];
    // joint_data_.effort[1] = msg->effort[1];
}

void JointDataLogger::timer_callback()
{
    std::stringstream data_stream_;
    data_stream_ << std::fixed << std::setprecision(4);

    rclcpp::Time msg_time_ = joint_data_.header.stamp;

    // rclcpp::Duration node_timestamp_ = this->get_clock()->now() - initial_time_;
    // if (node_timestamp_.seconds() > 1e-3){ }

    // data_ = "__time, header_timestamp, ";
    // data_ += "left_wheel_effort, left_wheel_position, left_wheel_velocity,";
    // data_ += "right_wheel_effort, right_wheel_position, right_wheel_velocity";

    // data_stream_ << std::to_string(this->get_clock()->now().seconds()) << ", "

    if (msg_time_.seconds() > 0.0)
    {
        timebase_ms_ += (1.0 / timer_frequency);
        data_stream_ << std::to_string(timebase_ms_) << ", "
                     << std::to_string(msg_time_.seconds()) << ", "
                     << joint_data_.effort[0] << ", "
                     << joint_data_.position[0] << ", "
                     << joint_data_.velocity[0] << ", "
                     << joint_data_.effort[1] << ", "
                     << joint_data_.position[1] << ", "
                     << joint_data_.velocity[1];

        if (!data_stream_.str().empty())
        {
            output_file_ << data_stream_.str() << std::endl;
            data_stream_.str().clear();
        }
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointDataLogger>());
    rclcpp::shutdown();
    return 0;
}