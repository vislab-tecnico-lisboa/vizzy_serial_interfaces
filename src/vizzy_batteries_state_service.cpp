#include "ros/ros.h"
#include <vizzy_msgs/BatteryState.h>
#include <segway_rmp/SegwayStatusStamped.h>
ros::ServiceClient kokam_battery_client;
ros::ServiceClient segway_battery_client;
bool query_state(vizzy_msgs::BatteryStateRequest &req,
                 vizzy_msgs::BatteryStateResponse &res)
{
    //battery_state(res.battery_state,res.percentage);
    vizzy_msgs::BatteryState kokam_srv_client;
    vizzy_msgs::BatteryState segway_srv_client;
    kokam_battery_client.call(kokam_srv_client);
    segway_battery_client.call(segway_srv_client);

    if (kokam_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::LOW_BATTERY || 
        segway_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::LOW_BATTERY){
            res.battery_state = vizzy_msgs::BatteryStateResponse::LOW_BATTERY;
    }
    else if (kokam_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::GOOD && 
        segway_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::GOOD){
            res.battery_state = vizzy_msgs::BatteryStateResponse::GOOD;
        }
    else if (kokam_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::CHARGED && 
        segway_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::GOOD){
            res.battery_state = vizzy_msgs::BatteryStateResponse::CHARGED;
        }
    else if (kokam_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::GOOD && 
        segway_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::CHARGED){
            res.battery_state = vizzy_msgs::BatteryStateResponse::CHARGED;
        }
    else if (kokam_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::MEDIUM ||
        segway_srv_client.response.battery_state == vizzy_msgs::BatteryStateResponse::MEDIUM){
            res.battery_state = vizzy_msgs::BatteryStateResponse::MEDIUM;
        }
    else{
        res.battery_state = vizzy_msgs::BatteryStateResponse::UNKNOWN;
    }

    ROS_INFO("sending back response: state [%ld]", res.battery_state);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vizzy_batteries_state_server");
    ros::NodeHandle n;
    kokam_battery_client = n.serviceClient<vizzy_msgs::BatteryState>("kokam_battery_state");
    segway_battery_client= n.serviceClient<vizzy_msgs::BatteryState>("segway_battery_state");
    ros::ServiceServer service = n.advertiseService("vizzy_batteries_state", query_state);
    ROS_INFO("Ready to check Vizzy's batteries state.");
    ros::spin();
    return 0;
}
