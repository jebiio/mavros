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

#include <kriso_msgs/LoggingStatus.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief KrisoStatusPlugin plugin.
 */
class LoggingStatusPlugin : public plugin::PluginBase {
public:
	LoggingStatusPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		LoggingStatus_sub = nh.subscribe("logging_status", 10, &LoggingStatusPlugin::logging_status_cb, this);
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

	ros::Subscriber LoggingStatus_sub;

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

	void logging_status_cb(const kriso_msgs::LoggingStatus::ConstPtr &req){
		//10Hz로 수신하여 GCS로 전송
		mavlink::kriso::msg::KRISO_ROS_LOG_STATUS kStatus{};
		
		kStatus.logging_status = req->logging_status;

		UAS_GCS(m_uas)->send_message_ignore_drop(kStatus);
		
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::LoggingStatusPlugin, mavros::plugin::PluginBase)