/**
 * @brief KRISO Control Command plugin
 * @file kriso_control_command.cpp
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

#include <mavros_msgs/KrisoControlCommand.h>
namespace mavros {
namespace std_plugins {
/**
 * @brief KrisoControlCommand plugin.
 */
class KrisoControlCommandPlugin : public plugin::PluginBase {
public:
	KrisoControlCommandPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");

		kriso_contro_command_pub = nh.advertise<mavros_msgs::KrisoControlCommand>("kriso_control_command", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&KrisoControlCommandPlugin::handle_kriso_control_command),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher kriso_contro_command_pub;

	void handle_kriso_control_command(const mavlink::mavlink_message_t *msg, mavlink::common::msg::KRISO_CONTROL_COMMAND &kriso_contro_command)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::KrisoControlCommand>();
		ros_msg->header = m_uas->synchronized_header(frame_id, kriso_contro_command.time_usec);

		ros_msg->op_mode = kriso_contro_command.op_mode;
		ros_msg->ca_mode = kriso_contro_command.ca_mode;
		ros_msg->ca_method = kriso_contro_command.ca_method;
		
		kriso_contro_command_pub.publish(ros_msg);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::KrisoControlCommandPlugin, mavros::plugin::PluginBase)
