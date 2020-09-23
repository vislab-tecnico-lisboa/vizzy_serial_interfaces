#include "ros/ros.h"s
#include <vizzy_msgs/BatteryState.h>
#include <segway_rmp/SegwayStatusStamped.h>
std::string *segway_topic;
double charged_voltage_threshold;
double low_battery_threshold;
double min_voltage = 60.0;
double max_voltage = 70.2;

uint8_t battery_state_logic(double voltage)
{
  std::cout << "Voltage :" << voltage << " low_battery_threshold : " << low_battery_threshold << std::endl;
    if (voltage > charged_voltage_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::CHARGED;
    }
    else if (voltage <= charged_voltage_threshold && voltage > low_battery_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::GOOD;
    }
    else if (voltage <= low_battery_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::LOW_BATTERY;
    }
    else
	return vizzy_msgs::BatteryStateResponse::UNKNOWN;
}

uint8_t battery_state(uint8_t &state,uint8_t &percentage)
{
    //Compute state

    double voltage;
    double current;
    boost::shared_ptr<segway_rmp::SegwayStatusStamped const> segway_status;
  segway_status = ros::topic::waitForMessage<segway_rmp::SegwayStatusStamped>(*segway_topic, ros::Duration(5));
  if (!segway_status)
  {
      state =  vizzy_msgs::BatteryStateResponse::UNKNOWN;
      percentage = 101;
  }
    else
    {
        state = battery_state_logic(segway_status->segway.powerbase_battery);
	percentage = floor(100.0*(segway_status->segway.powerbase_battery-min_voltage)/(max_voltage-min_voltage));
    }
}

bool query_state(vizzy_msgs::BatteryStateRequest &req,
                 vizzy_msgs::BatteryStateResponse &res)
{
    battery_state(res.battery_state,res.percentage);
    ROS_INFO("sending back response: state [%ld] percentage [%d]", res.battery_state,res.percentage);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "segway_battery_state_server");
    if (argc != 4)
    {
        ROS_INFO("usage: segway_battery_state_server </segway_topic> <charge_tresh> <low_batt_tresh>");
        return 1;
    }
    ros::NodeHandle n;
    segway_topic = new std::string(argv[1]);
    charged_voltage_threshold = atof(argv[2]);
    low_battery_threshold = atof(argv[3]);
    boost::shared_ptr<segway_rmp::SegwayStatusStamped const> segway_status;
  segway_status = ros::topic::waitForMessage<segway_rmp::SegwayStatusStamped>(*segway_topic, ros::Duration(5));
  if (!segway_status)
  {
      ROS_ERROR("Unable to connect to segway, check the buttons and the ROS node.");
  }
  else{
      std::cout << "Segway topic: " << *segway_topic << " Charged threshold: " << charged_voltage_threshold
             << " Low battery threshold: " << low_battery_threshold << std::endl;
        ros::ServiceServer service = n.advertiseService("segway_battery_state", query_state);
        ROS_INFO("Ready to check the Segway battery state.");
        ros::spin();
  }
    return 0;
}
