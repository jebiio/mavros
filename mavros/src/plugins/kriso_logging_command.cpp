/**
 * @brief QGC to ROS Plugin
 * @file kriso_qgc_to_ros.cpp
 * @author Jeyong Shin <jeyong@subak.io>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2023, Jeyong Shin
 *
 */

#include <angles/angles.h>

#include <mavros/mavros_plugin.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>

#include <kriso_msgs/LoggingCmd.h>


namespace mavros {
namespace std_plugins {
/**
 * @brief QGC to ROS plugin.
 *
 *
 */
class LoggingCommandPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	LoggingCommandPlugin() : PluginBase(),
		gp_nh("/kriso")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// general params
		// gp_nh.param<std::string>("kriso_data", frame_id, "map");		

		// UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		qgc_to_ros_pub = gp_nh.advertise<kriso_msgs::LoggingCmd>("logging_cmd", 5);

	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&LoggingCommandPlugin::handle_logging_command)
				// make_handler(&GlobalPositionPlugin::handle_gps_raw_int),
				// // GPS_STATUS: there no corresponding ROS message, and it is not supported by APM
				// make_handler(&GlobalPositionPlugin::handle_global_position_int),
				// make_handler(&GlobalPositionPlugin::handle_gps_global_origin),
				// make_handler(&GlobalPositionPlugin::handle_lpned_system_global_offset)
		};
	}

private:
	ros::NodeHandle gp_nh;

	ros::Publisher qgc_to_ros_pub;

	/* -*- message handlers -*- */

	void handle_logging_command(const mavlink::mavlink_message_t *msg, mavlink::kriso::msg::KRISO_ROS_LOG_COMMAND
 &command)
	{
		auto data = boost::make_shared<kriso_msgs::LoggingCmd>();
		data->logging_cmd = command.ros_log_command;

		qgc_to_ros_pub.publish(data);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LoggingCommandPlugin, mavros::plugin::PluginBase)
