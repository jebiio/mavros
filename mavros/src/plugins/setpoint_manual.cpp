/**
 * @brief SetpointManual plugin
 * @file setpoint_manual.cpp
 * @author jeyong <jeyong@subak.io>
 * 
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2022 Jeyong Shin.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ManualSetpoint.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Setpoint Manual plugin
 *
 * Send setpoint positions to FCU controller.
 */
class SetpointManualPlugin : public plugin::PluginBase {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SetpointManualPlugin() : PluginBase(),
		nh("~")
	{ }

    void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		manual_pub = nh.advertise<geometry_msgs::SetpointManual>("setpoint_manual", 20);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&SetpointManualPlugin::handle_setpoint_manual),
		};
	}

private:
	ros::NodeHandle nh;

	ros::Publisher manual_pub;

	void handle_setpoint_manual(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MANUAL_SETPOINT &manual)
	{
		//auto manual_setpoint_msg = boost::make_shared<px4control_msgs::ManualSetpoint>();
        //manual_setpoint_msg->header.stamp = m_uas->synchronise_stamp(wind.time_usec);
        manual_setpoint_msg.throttle = manual.thrust;
        manual_setpoint_msg.steering = manual.yaw;
        
		manual_pub.publish(manual_setpoint_msg);
	}

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::SetpointManualPlugin, mavros::plugin::PluginBase)
