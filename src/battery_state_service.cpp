#include "ros/ros.h"
#include <vizzy_msgs/BatteryState.h>
#include <vizzy_sensors/VoltageCurrentSerialInterface.hpp>

std::string *sensor_port;
double charged_voltage_threshold;
double low_battery_threshold;
VoltageCurrentSerialInterface *voltage_reader;
double min_voltage = 22.0;
double max_voltage = 29.4;

uint8_t battery_state_logic(double current, double voltage)
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

void battery_state(uint8_t &state,uint8_t &percentage)
{
    //Compute state

    double voltage;
    double current;
    if (voltage_reader->getSystemPowerSupply(current, voltage) == 0)
    {
        state = battery_state_logic(current, voltage);
        percentage = floor(100.0*voltage/max_voltage);
    }
    else
    {
        state = vizzy_msgs::BatteryStateResponse::UNKNOWN;
        percentage = 101;
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
    ros::init(argc, argv, "kokam_battery_state_server");
    if (argc != 4)
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
