/**
 * @brief Krisok Status plugin
 * @file kriso_status.cpp
 * @author Jeyong Shin <jeyong@subak.io>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2022 Jeyong Shin <jeyong@subak.io>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <kriso_msgs/MiddlewareToVcc.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief KrisoStatusPlugin plugin.
 */
// kriso의 상태를 받아서 GCS로 전송하는 Plugin
class KrisoStatusPlugin : public plugin::PluginBase {
public:
	KrisoStatusPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		kriso_status_sub = nh.subscribe("middleware_to_vcc", 10, &KrisoStatusPlugin::kriso_status_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		//ROS_WARN_NAMED("time", "KRISO : KRISO STATUS Received");

		//UAS_GCS(m_uas)->send_message_ignore_drop(nullptr);
		return {
			//make_handler(&KrisoStatusPlugin::handle_kriso_status),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Subscriber kriso_status_sub;

	// void kriso_status_cb(const mavros_msgs::KrisoStatus::ConstPtr &req)
	// {
	// 	ROS_WARN_NAMED("time", "KRISO : KRISO STATUS Received");
	// 	mavlink::common::msg::HIL_STATE hState {};
	// 	mavlink::kriso::msg::KRISO_STATUS kStatus{};

	// 	UAS_GCS(m_uas)->send_message_ignore_drop(hState);
	// }

	// void hil_state_cb(const mavros_msgs::KrisoStatus::ConstPtr &req)
	// {
	// 	//10Hz로 수신하여 GCS로 전송
	// 	mavlink::common::msg::HIL_STATE hState {};
	// 	mavlink::kriso::msg::KRISO_STATUS kStatus{};
	// 	UAS_GCS(m_uas)->send_message_ignore_drop(hState);
	// }

	// kriso 상태를 받아서 GCS로 전송. /kriso/middleware_to_vcc topic에서 수신한다.
	void kriso_status_cb(const kriso_msgs::MiddlewareToVcc::ConstPtr &req){
		//10Hz로 수신하여 GCS로 전송
		mavlink::kriso::msg::KRISO_STATUS kStatus{};

		kStatus.nav_mode = req->nav_mode;
		kStatus.nav_roll = req->nav_roll;
		kStatus.nav_pitch = req->nav_pitch;
		kStatus.nav_yaw = req->nav_yaw;
		kStatus.nav_yaw_rate = req->nav_yaw_rate;
		
		kStatus.nav_cog = req->nav_cog;
		kStatus.nav_sog = req->nav_sog;
		kStatus.nav_uspd = req->nav_uspd;
		kStatus.nav_vspd = req->nav_vspd;
		kStatus.nav_wspd = req->nav_wspd;
		
		kStatus.nav_longitude = req->nav_longitude;
		kStatus.nav_latitude = req->nav_latitude;

		kStatus.nav_heave = req->nav_heave;
		
		kStatus.nav_gpstime = req->nav_gpstime;
		kStatus.wea_airtem = req->wea_airtem;
		kStatus.wea_wattem = req->wea_wattem; 
		kStatus.wea_press  = req->wea_press;
		kStatus.wea_relhum = req->wea_relhum;
		
		kStatus.wea_dewpt = req->wea_dewpt;
		
		kStatus.wea_windirt = req->wea_windirt;
		kStatus.wea_winspdt = req->wea_winspdt;
		kStatus.wea_windirr = req->wea_windirr;
		kStatus.wea_watspdr = req->wea_watspdr;
		
		kStatus.wea_watdir = req->wea_watdir;
		kStatus.wea_watspd = req->wea_watspd;
		kStatus.wea_visibiran = req->wea_visibiran;

		UAS_GCS(m_uas)->send_message_ignore_drop(kStatus);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::KrisoStatusPlugin, mavros::plugin::PluginBase)
