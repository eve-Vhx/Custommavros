#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <std_msgs/Char.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_pkg/server_px4_reqAction.h>
#include <msg_pkg/server_px4_req_waypointsAction.h>
#include <msg_pkg/connections_drone.h>
#include <msg_pkg/feedbackMsg.h>
#include <msg_pkg/serv_mission_st.h>
#include <string>
#include "ros/ros.h"
//#include <mavros_extras/smr_msg.h>
namespace mavros {
namespace extra_plugins{

class ServerPx4RoutePlugin : public plugin::PluginBase {


public:
    ServerPx4RoutePlugin() : 
        PluginBase(), 
        nh("~smr_px4_command"),
        as_(nh, "d1_cmd_action", boost::bind(&ServerPx4RoutePlugin::actionCB, this, _1), false),
        as_waypoints_(nh, "d1_cmd_action_waypoints", boost::bind(&ServerPx4RoutePlugin::WaypointActionCB, this, _1), false)
    { 
        as_.start();
        as_waypoints_.start();
        srv_m_st_pub = nh.advertise<msg_pkg::serv_mission_st>("server_mission_state", 10);
        feedback_pub = nh.advertise<msg_pkg::feedbackMsg>("pi_action_feedback", 10);
    };


    ~ServerPx4RoutePlugin(void)
    {        
    }


    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
    };

    void WaypointActionCB(const msg_pkg::server_px4_req_waypointsGoalConstPtr &waypoint_goal)
    {
        ROS_INFO("Inside the waypoints callback");

        mavlink::server_mission_request::msg::SERVER_MISSION_WAYPOINTS_REQUEST waypoints_smr {};
        waypoints_smr.mission_type = waypoint_goal->mission_type;

        waypoints_smr.lon = waypoint_goal->dest_lon;
        waypoints_smr.lat = waypoint_goal->dest_lat;
        waypoints_smr.alt = waypoint_goal->dest_alt;
        
        waypoints_smr.yaw = waypoint_goal->yaw_rad;
        waypoints_smr.cruise_alt = waypoint_goal->cruise_alt;
        waypoints_smr.timestamp = waypoint_goal->timestamp;
        for (int i=0; i < waypoint_goal->lat.size(); i++) {
            waypoints_smr.waypoints[3*i] = waypoint_goal->lat[i];
            waypoints_smr.waypoints[3*i + 1] = waypoint_goal->lon[i];
            waypoints_smr.waypoints[3*i + 2] = waypoint_goal->alt[i];
            std::cout<< "Waypoint object: "<< waypoint_goal->lat[i] << "," << waypoint_goal->lon[i] << "," << waypoint_goal->alt[i] << std::endl;
        }
        UAS_FCU(m_uas)->send_message_ignore_drop(waypoints_smr);

        waypoints_result_.mission_req_status = 1;
        as_waypoints_.setSucceeded(waypoints_result_);
    }

    void actionCB(const msg_pkg::server_px4_reqGoalConstPtr &goal)
    {
        //Rpi action server has received request from server code: 0
        feedback_msg.feedback = 0;
        feedback_msg.drone_id = "QROW11021";
        feedback_pub.publish(feedback_msg);
        ROS_INFO("Inside action callback");
        mavlink::server_mission_request::msg::SERVER_MISSION_REQUEST smr {};
        smr.mission_type = goal->mission_type;
        smr.lon = goal->lon;
        smr.lat = goal->lat;
        smr.alt = goal->alt;
        smr.yaw = goal->yaw_rad;
        smr.timestamp = goal->timestamp;
        smr.cruise_alt = goal->cruise_alt;
        UAS_FCU(m_uas)->send_message_ignore_drop(smr);
        //Rpi action server has sent through mission request to the UART for Px4 code: 1
        feedback_msg.feedback = 1;
        feedback_msg.drone_id = "QROW11021";
        feedback_pub.publish(feedback_msg);
        std::cout<< "Publishing to px4... Mission details | lat: "<< goal->lat << ", lon: " << goal->lon << ", alt: " << goal->alt << std::endl;

        ros::Rate r(1);
        bool success = true;

        if(success){
            result_.mission_req_status = 2;
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }


    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&ServerPx4RoutePlugin::handle_missionstate)
        };
    }
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<msg_pkg::server_px4_reqAction> as_;
    msg_pkg::server_px4_reqFeedback feedback_;
    msg_pkg::server_px4_reqResult result_;

    actionlib::SimpleActionServer<msg_pkg::server_px4_req_waypointsAction> as_waypoints_;
    msg_pkg::server_px4_req_waypointsFeedback waypoints_feedback_;
    msg_pkg::server_px4_req_waypointsResult waypoints_result_;

    ros::Publisher srv_m_st_pub;
    ros::Publisher feedback_pub;

    msg_pkg::feedbackMsg feedback_msg;

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

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServerPx4RoutePlugin, mavros::plugin::PluginBase)