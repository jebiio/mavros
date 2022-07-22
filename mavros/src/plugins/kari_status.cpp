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

//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief KrisoStatusPlugin plugin.
 */
class KrisoStatusPlugin : public plugin::PluginBase {
public:
	KrisoStatusPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		//kriso_status_sub = sp_nh.subscribe("local", 10, &KrisoStatusPlugin::kriso_status_cb, this);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			//make_handler(&KrisoStatusPlugin::handle_kriso_status),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Subscriber kriso_status_sub;
//	ros::Publisher kriso_status_pub;

};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::KrisoStatusPlugin, mavros::plugin::PluginBase)
