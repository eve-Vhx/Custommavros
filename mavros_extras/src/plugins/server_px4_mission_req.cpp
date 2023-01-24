#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <std_msgs/Char.h>
#include <actionlib/server/simple_action_server.h>
#include <msg_pkg/server_px4_reqAction.h>
#include <string>
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
        mavlink::common::msg::SERVER_MISSION_REQUEST smr {};
        smr.mission_type = goal->mission_type;
        smr.lon = goal->lon;
        smr.lat = goal->lat;
        smr.alt = goal->alt;
        smr.yaw = goal->yaw_rad;
        smr.timestamp = goal->timestamp;
        UAS_FCU(m_uas)->send_message_ignore_drop(smr);
        std::cout<< "Publishing to px4... Mission details | lat: "<< goal->lat << ", lon: " << goal->lon << ", alt: " << goal->alt << std::endl;

        ros::Rate r(1);
        bool success = true;

        if(success){
            result_.mission_completion = 1;
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }

    Subscriptions get_subscriptions() override
    {
        return {
            make_handler(&ServerPx4MissionRequestPlugin::handle_missionrequest)
        };
    }
private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<msg_pkg::server_px4_reqAction> as_;
    msg_pkg::server_px4_reqFeedback feedback_;
    msg_pkg::server_px4_reqResult result_;
    ros::Publisher rangefinder_pub;

    void handle_missionrequest(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVER_MISSION_REQUEST &srv_m) {
        auto srv_m_msg = boost::make_shared<sensor_msgs::Range>();
        srv_m_msg->timestamp = srv_m.timestamp;
        srv_m_msg->mission_type = srv_m.mission_type;
        srv_m_msg->lat = srv_m.lat;
        srv_m_msg->lon = srv_m.lon;
        srv_m_msg->alt = srv_m.alt;
        srv_m_msg->yaw = srv_m.yaw;
        srv_m_pub.publish(srv_m_msg);
    }

};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServerPx4MissionRequestPlugin, mavros::plugin::PluginBase)