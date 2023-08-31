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

#include <kriso_msgs/MTCmd.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief QGC to ROS plugin.
 *
 *
 */
class MtCommandPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	MtCommandPlugin() : PluginBase(),
		gp_nh("/kriso")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// general params
		// gp_nh.param<std::string>("kriso_data", frame_id, "map");		

		// UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		qgc_to_ros_pub = gp_nh.advertise<kriso_msgs::MTCmd>("mt_cmd", 5);

	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&MtCommandPlugin::handle_mt_command)
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

	void handle_mt_command(const mavlink::mavlink_message_t *msg, mavlink::kriso::msg::KRISO_MT_COMMAND &command)
	{
		auto data = boost::make_shared<kriso_msgs::MTCmd>();
		data->start		= command.start;
		data->t1_rpm	= command.t1_rpm;
		data->t2_rpm	= command.t2_rpm;
		data->t3_rpm	= command.t3_rpm;
		data->t3_angle	= command.t3_angle;
		data->t4_rpm	= command.t4_rpm;
		data->t4_angle	= command.t4_angle;
		
		qgc_to_ros_pub.publish(data);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MtCommandPlugin, mavros::plugin::PluginBase)
