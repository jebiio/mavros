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

#include <kriso_msgs/PlcToVcc.h>
//#include <mavros_msgs/krisoStatus.h>

namespace mavros {
namespace std_plugins {
using mavlink::common::MAV_FRAME;
/**
 * @brief plc_to_vcc  plugin.
 */
// PLC에서 plc_to_vcc를 보내면, vcc에서 QGC로 전송
class PlcToVccPlugin : public plugin::PluginBase {
public:
	PlcToVccPlugin() : PluginBase(),
		nh("/kriso")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);
		nh.param<std::string>("frame_id", frame_id, "map");
		
		PlcToVcc_sub = nh.subscribe("plc_to_vcc", 10, &PlcToVccPlugin::plc_to_vcc_cb, this);
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

	ros::Subscriber PlcToVcc_sub;

	// /kriso/plc_to_vcc topic상에서 PlcToVcc 수신하여 QGC로 전송
	void plc_to_vcc_cb(const kriso_msgs::PlcToVcc::ConstPtr &req){
		
		mavlink::kriso::msg::KRISO_PLC_TO_VCC kStatus{};

		kStatus.mr_mtr_sta		 = req->mr_mtr_sta		
		kStatus.mr_flt_msg_err1	 = req->mr_flt_msg_err1	
		kStatus.mr_flt_msg_err2	 = req->mr_flt_msg_err2	
		kStatus.mr_flt_msg_warn1 = req->mr_flt_msg_warn1
		kStatus.mr_flt_msg_warn2 = req->mr_flt_msg_warn2
		kStatus.mr_mtr_curr_real = req->mr_mtr_curr_real
		kStatus.mr_temp		 	 = req->mr_temp		 	
		kStatus.mr_mtr_rpm_real	 = req->mr_mtr_rpm_real	
		kStatus.mr_mtr_rot_real	 = req->mr_mtr_rot_real	
		kStatus.ml_mtr_sta	 	 = req->ml_mtr_sta	 	
		kStatus.ml_flt_msg_err1	 = req->ml_flt_msg_err1	
		kStatus.ml_flt_msg_err2	 = req->ml_flt_msg_err2	
		kStatus.ml_flt_msg_warn1 = req->ml_flt_msg_warn1
		kStatus.ml_flt_msg_warn2 = req->ml_flt_msg_warn2
		kStatus.ml_mtr_curr_real = req->ml_mtr_curr_real
		kStatus.ml_temp	 	 	 = req->ml_temp	 	 	
		kStatus.ml_mtr_rpm_real	 = req->ml_mtr_rpm_real	
		kStatus.ml_mtr_rot_real	 = req->ml_mtr_rot_real	
		kStatus.br_mtr_sta	 	 = req->br_mtr_sta	 	
		kStatus.br_flt_msg	 	 = req->br_flt_msg	 	
		kStatus.br_mtr_curr_real = req->br_mtr_curr_real
		kStatus.br_temp	 	 	 = req->br_temp	 	 	
		kStatus.br_mtr_rpm	 	 = req->br_mtr_rpm	 	
		kStatus.br_mtr_rot_sta	 = req->br_mtr_rot_sta	
		kStatus.bl_mtr_sta	 	 = req->bl_mtr_sta	 	
		kStatus.bl_flt_msg	 	 = req->bl_flt_msg	 	
		kStatus.bl_mtr_curr_real = req->bl_mtr_curr_real
		kStatus.bl_temp		 	 = req->bl_temp		 	
		kStatus.bl_mtr_rpm	 	 = req->bl_mtr_rpm	 	
		kStatus.bl_mtr_rot_sta	 = req->bl_mtr_rot_sta	
		kStatus.sr_str_rpm	 	 = req->sr_str_rpm	 	
		kStatus.sr_str_ang		 = req->sr_str_ang		
		kStatus.sl_str_rpm		 = req->sl_str_rpm		
		kStatus.sl_str_ang	 	 = req->sl_str_ang	 	
		kStatus.batt400vdc		 = req->batt400vdc		
		kStatus.batt24vdc_1		 = req->batt24vdc_1		
		kStatus.batt24vdc_2		 = req->batt24vdc_2		

		// QGC로 전송하는 부분
		UAS_GCS(m_uas)->send_message_ignore_drop(kStatus);
	}
};
}	// namespace std_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::PlcToVccPlugin, mavros::plugin::PluginBase)
