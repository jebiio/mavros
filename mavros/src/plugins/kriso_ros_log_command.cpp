/**
 * @brief KRISO ROS Log Command plugin
 * @file kriso_ros_log_command.cpp
 * @author Jeyong Shin <jeyong@subak.io>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2023 Jeyong Shin <jeyong@subak.io>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/KrisoRosLogCommand.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief KrisoRosLogCommand plugin.
 */
class KrisoRosLogCommandPlugin : public plugin::PluginBase {
public:
	KrisoRosLogCommandPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		kriso_ros_log_command_pub = nh.advertise<mavros_msgs::KrisoRosLogCommand>("krisoroslogcommand", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&KrisoRosLogCommandPlugin::handle_kriso_ros_log_command),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher kriso_ros_log_command_pub;

	void handle_kriso_ros_log_command(const mavlink::mavlink_message_t *msg, mavlink::common::msg::KRISO_ROS_LOG_COMMAND &kriso_ros_log_command)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::KrisoRosLogCommand>();
		ros_msg->header = m_uas->synchronized_header(frame_id, kriso_ros_log_command.time_usec);

		ros_msg->ros_log_command = kriso_ros_log_command.ros_log_command;

		kriso_ros_log_command_pub.publish(ros_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::KrisoRosLogCommandPlugin, mavros::plugin::PluginBase)
