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

class ServerPx4RequestPlugin : public plugin::PluginBase {

protected:
    actionlib::SimpleActionServer<msg_pkg::server_px4_reqAction> as_;
    std::string action_name_;
    msg_pkg::server_px4_reqFeedback feedback_;
    msg_pkg::server_px4_reqResult result_;

public:
    ServerPx4RequestPlugin() : 
        PluginBase(), 
        as_(nh, "d1_cmd_action", boost::bind(&ServerPx4RequestPlugin::actionCB, this, _1), false),
        action_name_("d1_cmd_action"),
        nh("~smr_command")

    { 
        as_.start();
    };

    ~ServerPx4RequestPlugin(void)
    {        
    }


    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
    };

    void actionCB(const msg_pkg::server_px4_reqGoalConstPtr &goal)
    {

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

     Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;


};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServerPx4MissionRequestPlugin, mavros::plugin::PluginBase)