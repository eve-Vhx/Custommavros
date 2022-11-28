 #include <mavros/mavros_plugin.h>
 #include <pluginlib/class_list_macros.h>
 #include <iostream>
 #include <std_msgs/Char.h>
 #include <msg_pkg/serv_mission_req.h>
//#include <mavros_extras/smr_msg.h>
 namespace mavros {
 namespace extra_plugins{

 class ServerMissionRequestPlugin : public plugin::PluginBase {
 public:
     ServerMissionRequestPlugin() : PluginBase(),
         nh("~smr_command")

    { };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         smr_sub = nh.subscribe("smr", 10, &ServerMissionRequestPlugin::smr_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }

 private:
     ros::NodeHandle nh;
     ros::Subscriber smr_sub;

    void smr_cb(const msg_pkg::serv_mission_req::ConstPtr &req)
     {
         mavlink::common::msg::SERVER_MISSION_REQUEST smr {};
         smr.mission_type = req->mission_type;
         smr.lon = req->lon;
         smr.lat = req->lat;
         smr.alt = req->alt;
         smr.yaw = req->yaw_rad;
         smr.timestamp = req->timestamp;
         UAS_FCU(m_uas)->send_message_ignore_drop(smr);
         std::cout<< "Message sent" << std::endl;
     }
 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServerMissionRequestPlugin, mavros::plugin::PluginBase)
