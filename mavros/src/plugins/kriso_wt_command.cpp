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

#include <kriso_msgs/WTtoController.h>


namespace mavros {
namespace std_plugins {
/**
 * @brief QGC to ROS plugin.
 *
 *
 */
//  WT_Command 메시지를 QGC로부터 받아서 ROS로 전송하는 Plugin
class WtCommandPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	WtCommandPlugin() : PluginBase(),
		gp_nh("/kriso")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		// general params
		// gp_nh.param<std::string>("kriso_data", frame_id, "map");		

		// UAS_DIAG(m_uas).add("GPS", this, &GlobalPositionPlugin::gps_diag_run);

		qgc_to_ros_pub = gp_nh.advertise<kriso_msgs::WTtoController>("wt_to_controller", 5);

	}

	Subscriptions get_subscriptions() override
	{
		return {
				make_handler(&WtCommandPlugin::handle_wt_command)
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

	// wt_to_controller 메시지를 QGC로부터 받아서 ROS로 전송하는 함수. /kriso/wt_to_controller topic으로 전송
	void handle_wt_command(const mavlink::mavlink_message_t *msg, mavlink::kriso::msg::KRISO_WT_COMMAND &command)
	{
		ROS_INFO("mavlink WT_Command received!!");
		auto data = boost::make_shared<kriso_msgs::WTtoController>();

		for(int i=0; i<5; i++){ //현재 5개만 가능
			data->global_path[i].lat = 	command.lat[i];
			data->global_path[i].lon = command.lon[i];
			data->global_path[i].spd_cmd = command.spd_cmd[i];
			data->global_path[i].acceptance_radius = command.acceptance_radius[i];
		}
		data->nav_surge_pgain = command.nav_surge_pgain;
		data->nav_surge_dgain = command.nav_surge_dgain;
		data->nav_yaw_pgain = command.nav_yaw_pgain;
		data->nav_yaw_dgain = command.nav_yaw_dgain;
		data->count = command.count;

		qgc_to_ros_pub.publish(data);
		ROS_INFO("ROS WTtoController published!!!!");
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WtCommandPlugin, mavros::plugin::PluginBase)
