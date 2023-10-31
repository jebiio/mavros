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

#include <kriso_msgs/WTtoVcc.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief KrisoStatusPlugin plugin.
 */
// wt_to_vcc 메시지를 GCS로 전송하는 Plugin
class WtToVccPlugin : public plugin::PluginBase {
public:
	WtToVccPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		WttoVcc_sub = nh.subscribe("wt_to_vcc", 10, &WtToVccPlugin::wt_to_vcc_cb, this);
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

	ros::Subscriber WttoVcc_sub;

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

	// wt_to_vcc 메시지를 수신하여 GCS로 전송. /kriso/wt_to_vcc topic으로부터 수신한다.
	void wt_to_vcc_cb(const kriso_msgs::WTtoVcc::ConstPtr &req){
		//10Hz로 수신하여 GCS로 전송
		mavlink::kriso::msg::KRISO_WT_TO_VCC kStatus{};
		
		kStatus.psi_d = req->psi_d;
		kStatus.spd_d = req->spd_d;
		kStatus.wp_lat_d = req->wp_lat_d;
		kStatus.wp_lon_d = req->wp_lon_d;
		kStatus.track_path_idx = req->track_path_idx;

		UAS_GCS(m_uas)->send_message_ignore_drop(kStatus);
		
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::WtToVccPlugin, mavros::plugin::PluginBase)
