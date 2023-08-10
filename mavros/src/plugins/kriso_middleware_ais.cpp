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
#include <kriso_msgs/MiddlewareAIS.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief KrisoStatusPlugin plugin.
 */
class KrisoMiddlewareAISPlugin : public plugin::PluginBase {
public:
	KrisoMiddlewareAISPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		kriso_middleware_ais_sub = nh.subscribe("middleware_ais", 10, &KrisoMiddlewareAISPlugin::kriso_middleware_ais_cb, this);
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

	ros::Subscriber kriso_middleware_ais_sub;

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

	void kriso_middleware_ais_cb(const kriso_msgs::MiddlewareAIS::ConstPtr &req){
		//10Hz로 수신하여 GCS로 전송
		mavlink::kriso::msg::KRISO_AIS_STATUS kStatus{};

		kStatus.msg_type = req->msg_type;
		kStatus.repeat = req->repeat;
		kStatus.mmsi = req->mmsi;
		kStatus.reserved_1 = req->reserved_1;
		kStatus.speed = req->speed;
		kStatus.accuracy = req->accuracy;
		kStatus.lon = req->lon;
		kStatus.lat = req->lat;
		kStatus.course = req->course;
		kStatus.heading = req->heading;
		kStatus.second = req->second;
		kStatus.reserved_2 = req->reserved_2;
		kStatus.cs = req->cs;
		kStatus.display = req->display;
		kStatus.dsc = req->dsc;
		kStatus.band = req->band; 
		kStatus.msg22  = req->msg22;
		kStatus.assigned = req->assigned;
		kStatus.raim = req->raim;
		kStatus.radio = req->radio;

		UAS_GCS(m_uas)->send_message_ignore_drop(kStatus);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::KrisoMiddlewareAISPlugin, mavros::plugin::PluginBase)
