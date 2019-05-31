#include "ros/ros.h"
#include <vizzy_msgs/BatteryState.h>
#include <vizzy_sensors/VoltageCurrentSerialInterface.hpp>

std::string *sensor_port;
double charged_voltage_threshold;
double low_battery_threshold;

uint8_t battery_state_logic(double voltage)
{
    if (voltage > charged_voltage_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::CHARGED;
    }
    else if (voltage < charged_voltage_threshold && voltage > low_battery_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::GOOD;
    }
    else if (voltage < low_battery_threshold)
    {
        return vizzy_msgs::BatteryStateResponse::LOW_BATTERY;
    }
}

uint8_t battery_state()
{
    //Compute state

    double voltage;
    double current;
    if (voltage_reader->getSystemPowerSupply(current, voltage) == 0)
    {
        return battery_state_logic(current, voltage);
    }
    else
    {
        return vizzy_msgs::BatteryStateResponse::UNKNOWN;
    }
}

bool query_state(vizzy_msgs::BatteryStateRequest &req,
                 vizzy_msgs::BatteryStateResponse &res)
{
    res.battery_state = battery_state();
    ROS_INFO("sending back response: [%ld]", res.battery_state);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kokam_battery_state_server");
    if (argc != 5)
    {
        ROS_INFO("usage: kokam_battery_state_server </dev/ttyUSBX> <charge_tresh> <low_batt_tresh>");
        return 1;
    }
    ros::NodeHandle n;
    voltage_reader = new VoltageCurrentSerialInterface();
    sensor_port = new std::string(argv[1]);
    charged_voltage_threshold = atof(argv[2]);
    low_battery_threshold = atof(argv[3]);
    if (voltage_reader->initComm(*sensor_port))
    {
        cout << "Port: " << *sensor_port << " Charged threshold: " << charged_voltage_threshold
             << " Low battery threshold: " << low_battery_threshold << endl;
        ros::ServiceServer service = n.advertiseService("kokam_battery_state", query_state);
        ROS_INFO("Ready to check the Kokam battery state.");
        ros::spin();
    }
    else
    {
        ROS_ERROR("Unable to connect to the battery state PCB.");
    }
    voltage_reader->closeComm();
    return 0;
}