#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <std_msgs/Char.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_pkg/server_px4_reqAction.h>
#include <msg_pkg/connections_drone.h>
#include <msg_pkg/serv_mission_st.h>
#include <string>
#include "ros/ros.h"
//#include <mavros_extras/smr_msg.h>
namespace mavros {
namespace extra_plugins{

class ServerPx4MissionRequestPlugin : public plugin::PluginBase {


public:
    ServerPx4MissionRequestPlugin() : 
        PluginBase(), 
        nh("~smr_px4_command"),
        as_(nh, "d1_cmd_action", boost::bind(&ServerPx4MissionRequestPlugin::actionCB, this, _1), false)

    { 
        as_.start();
        connections_sub = nh.subscribe("/d1_connection_checks", 10, &ServerPx4MissionRequestPlugin::connectionCB, this);
        srv_m_st_pub = nh.advertise<msg_pkg::serv_mission_st>("server_mission_state", 10);

    };


    ~ServerPx4MissionRequestPlugin(void)
    {        
    }


    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
    };

    void actionCB(const msg_pkg::server_px4_reqGoalConstPtr &goal)
    {
        ROS_INFO("Inside action callback");
        if(px4_status == true) {
            ROS_INFO("Passed Rpi connection checks... Sending mission");
            mavlink::server_mission_request::msg::SERVER_MISSION_REQUEST smr {};
            smr.mission_type = goal->mission_type;
            smr.lon = goal->lon;
            smr.lat = goal->lat;
            smr.alt = goal->alt;
            smr.yaw = goal->yaw_rad;
            smr.timestamp = goal->timestamp;
            smr.cruise_alt = goal->cruise_alt;
            UAS_FCU(m_uas)->send_message_ignore_drop(smr);
            std::cout<< "Publishing to px4... Mission details | lat: "<< goal->lat << ", lon: " << goal->lon << ", alt: " << goal->alt << std::endl;

            ros::Rate r(1);
            bool success = true;

            if(success){
                result_.mission_req_status = 1;
                // set the action state to succeeded
                as_.setSucceeded(result_);
            }
        }
        else { ROS_INFO("Cannot send mission to Px4. Connection check failed.");}
    }

    void connectionCB(const msg_pkg::connections_drone::ConstPtr& msg) {
        px4_status = msg->px4;
        //std::cout << "PX4 Status: " << px4_status << std::endl;
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&ServerPx4MissionRequestPlugin::handle_missionstate)
        };
    }
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<msg_pkg::server_px4_reqAction> as_;
    msg_pkg::server_px4_reqFeedback feedback_;
    msg_pkg::server_px4_reqResult result_;
    ros::Subscriber connections_sub;
    bool px4_status;

    ros::Publisher srv_m_st_pub;

    void handle_missionstate(const mavlink::mavlink_message_t *msg, mavlink::server_mission_request::msg::SERVER_MISSION_STATE &srv_m_st) {
        auto srv_m_st_msg = boost::make_shared<msg_pkg::serv_mission_st>();
        srv_m_st_msg->timestamp = srv_m_st.timestamp;
        srv_m_st_msg->mission_type = srv_m_st.mission_type;
        srv_m_st_msg->lat = srv_m_st.lat;
        srv_m_st_msg->lon = srv_m_st.lon;
        srv_m_st_msg->alt = srv_m_st.alt;
        srv_m_st_msg->yaw_rad = srv_m_st.yaw;
        srv_m_st_msg->state = srv_m_st.state;
        srv_m_st_msg->cruise_alt = srv_m_st.cruise_alt;
        srv_m_st_pub.publish(srv_m_st_msg);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServerPx4MissionRequestPlugin, mavros::plugin::PluginBase)