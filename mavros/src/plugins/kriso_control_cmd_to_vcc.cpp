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

#include <kriso_msgs/ControlcmdtoVcc.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief ControlCmdToVccPlugin plugin.
 */
// 
//  Controller에서 전송하는 control_cmd_to_vcc 메시지를 수신하여 GCS로 전송하는 Plugin
class ContolCmdToVccPlugin : public plugin::PluginBase {
public:
	ContolCmdToVccPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		ControlCmd_to_vcc_sub = nh.subscribe("control_cmd_to_vcc", 10, &ContolCmdToVccPlugin::control_cmd_to_vcc_cb, this);
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

	ros::Subscriber ControlCmd_to_vcc_sub;

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

	// control_cmd_to_vcc 메시지를 수신하여 GCS로 전송. /kriso/control_cmd_to_vcc topic으로 전송한다.
	void control_cmd_to_vcc_cb(const kriso_msgs::ControlcmdtoVcc::ConstPtr &req){
		//10Hz로 수신하여 GCS로 전송
		mavlink::kriso::msg::KRISO_CONTROL_COMMAND_TO_VCC kCmdToVcc{};
				
		kCmdToVcc.t1_rpm = req->t1_rpm; 
		kCmdToVcc.t2_rpm = req->t2_rpm;
		kCmdToVcc.t3_rpm = req->t3_rpm;
		kCmdToVcc.t3_angle = req->t3_angle;
		kCmdToVcc.t4_rpm = req->t4_rpm;
		kCmdToVcc.t4_angle = req->t4_angle;
		kCmdToVcc.oper_mode = req->oper_mode;
		kCmdToVcc.mission_mode = req->mission_mode;
		UAS_GCS(m_uas)->send_message_ignore_drop(kCmdToVcc);
		
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::ContolCmdToVccPlugin, mavros::plugin::PluginBase)
