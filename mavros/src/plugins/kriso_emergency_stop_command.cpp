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

#include <kriso_msgs/EmergencyStopCmd.h>


namespace mavros {
namespace std_plugins {
/**
 * @brief QGC to ROS plugin.
 *
 *
 */
// 긴급정지 메시지 emergency_stop_cmd를 QGC로부터 받아서 ROS로 전송하는 Plugin
class EmergencyStopCommandPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	EmergencyStopCommandPlugin() : PluginBase(),
		gp_nh("/kriso")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// general params
		// gp_nh.param<std::string>("kriso_data", frame_id, "map");		

		// UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		qgc_to_ros_pub = gp_nh.advertise<kriso_msgs::EmergencyStopCmd>("emergency_stop_cmd", 5);

	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&EmergencyStopCommandPlugin::handle_emergency_stop_command)
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

	// 긴급정지 메시지를 QGC로부터 수신하여 ROS로 전송. /kriso/emergency_stop_cmd topic으로 전송
	void handle_emergency_stop_command(const mavlink::mavlink_message_t *msg, mavlink::kriso::msg::KRISO_EMERGENCY_COMMAND
 &command)
	{
		auto data = boost::make_shared<kriso_msgs::EmergencyStopCmd>();
		data->emergency_cmd = command.emergency_mode;

		qgc_to_ros_pub.publish(data);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::EmergencyStopCommandPlugin, mavros::plugin::PluginBase)
