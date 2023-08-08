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

#include <kriso_msgs/CmdtoController.h>


namespace mavros {
namespace std_plugins {
/**
 * @brief QGC to ROS plugin.
 *
 *
 */
class CmdCommandPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CmdCommandPlugin() : PluginBase(),
		gp_nh("/kriso")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// general params
		// gp_nh.param<std::string>("kriso_data", frame_id, "map");		

		// UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		qgc_to_ros_pub = gp_nh.advertise<kriso_msgs::CmdtoController>("cmd_to_controller", 5);

	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&CmdCommandPlugin::handle_cmd_command)
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

	void handle_cmd_command(const mavlink::mavlink_message_t *msg, mavlink::kriso::msg::KRISO_CONTROL_COMMAND &command)
	{
		auto data = boost::make_shared<kriso_msgs::CmdtoController>();
		data->oper_mode = command.op_mode;
		data->mission_mode = command.mission_mode;
		data->ca_mode = command.ca_mode;
		data->ca_method = command.ca_method;

		qgc_to_ros_pub.publish(data);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::CmdCommandPlugin, mavros::plugin::PluginBase)
