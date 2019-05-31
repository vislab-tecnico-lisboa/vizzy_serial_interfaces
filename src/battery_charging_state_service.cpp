#include "ros/ros.h"
#include <vizzy_msgs/BatteryChargingState.h>
#include <vizzy_sensors/VoltageCurrentSerialInterface.hpp>
#include <segway_rmp/SegwayStatusStamped.h>
std::string *sensor_port;
std::string *segway_topic;

VoltageCurrentSerialInterface *voltage_reader;
int samples = 40;
bool check_segway()
{
  //segway_rmp::Path path = ros::topic::waitForMessage("/path_planned/edge");
  return false;
}

double l2_line_fitting(double x[], double y[])
{
  double xsum = 0.0, x2sum = 0.0, ysum = 0.0, xysum = 0.0;
  for (int i = 0; i < samples; i++)
  {
    xsum = xsum + x[i];           //calculate sigma(xi)
    ysum = ysum + y[i];           //calculate sigma(yi)
    x2sum = x2sum + pow(x[i], 2); //calculate sigma(x^2i)
    xysum = xysum + x[i] * y[i];  //calculate sigma(xi*yi)
  }
  double est_slope = (samples * xysum - xsum * ysum) / (samples * x2sum - xsum * xsum); //calculate slope
  double est_intercept = (x2sum * ysum - xsum * xysum) / (x2sum * samples - xsum * xsum);
  return est_slope;
}

uint8_t battery_charging_state()
{
  double voltage[40];
  double time_secs[40];
  double current;
  bool done_reading = false;
  int sample_counter = 0;
  double mean_voltage = 0.0;
  double current_voltage;
  double init_voltage = 0.0;
  double end_voltage = 0.0;
  double init_samples = 0.0;
  double end_samples = 0.0;
  std::cout << "Start reading values" << endl;
  double init_time = ros::Time::now().toSec();
  while (!done_reading)
  {
    if (voltage_reader->getSystemPowerSupply(current, current_voltage) == 0)
    {
      voltage[sample_counter] = current_voltage;
      time_secs[sample_counter] = ros::Time::now().toSec() - init_time;
      //std::cout << "Voltage: " << voltage[sample_counter] << " time: " << time_secs[sample_counter] << endl;
      //ros::Duration(0.05).sleep();
      if (sample_counter == samples)
        done_reading = true;
      mean_voltage += voltage[sample_counter];
      if (sample_counter < samples / 4)
      {
        init_voltage += voltage[sample_counter];
        init_samples++;
      }
      if (sample_counter > samples * 3 / 4 && sample_counter < samples)
      {
        end_voltage += voltage[sample_counter];
        end_samples++;
      }
      sample_counter++;
    }
    else
    {
      done_reading = true;
      return vizzy_msgs::BatteryChargingStateResponse::UNKNOWN;
    }
  }
  mean_voltage /= sample_counter;
  init_voltage /= init_samples;
  end_voltage /= end_samples;
  double slope = l2_line_fitting(time_secs, voltage);
  std::cout << "Slope: " << slope << endl;
  std::cout << "mean power: " << mean_voltage << " init voltage: " << init_voltage << " end voltage " << end_voltage << "delta: " << init_voltage - end_voltage << endl;
  if (slope > 0.0)
    return vizzy_msgs::BatteryChargingStateResponse::CHARGING;
  else
    return vizzy_msgs::BatteryChargingStateResponse::NOT_CHARGING;
}

bool query_charging_state(vizzy_msgs::BatteryChargingStateRequest &req,
                          vizzy_msgs::BatteryChargingStateResponse &res)
{
  std::cout << "Service called" << endl;
  res.battery_charging_state = battery_charging_state();
  return true;
}

bool query_segway_charging_state(vizzy_msgs::BatteryChargingStateRequest &req,
                                 vizzy_msgs::BatteryChargingStateResponse &res)
{
  boost::shared_ptr<segway_rmp::SegwayStatusStamped const> segway_status;
  segway_status = ros::topic::waitForMessage<segway_rmp::SegwayStatusStamped>(*segway_topic, ros::Duration(5));
  if (!segway_status)
  {
    res.battery_charging_state = vizzy_msgs::BatteryChargingStateResponse::UNKNOWN;
  }
  else
  {
    double current_voltage_charge = segway_status->segway.ui_battery;
    if (current_voltage_charge > 8.0)
      res.battery_charging_state = vizzy_msgs::BatteryChargingStateResponse::CHARGING;
    else if (current_voltage_charge < 7.0)
      res.battery_charging_state = vizzy_msgs::BatteryChargingStateResponse::NOT_CHARGING;
    else
      res.battery_charging_state = vizzy_msgs::BatteryChargingStateResponse::UNKNOWN;
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_charging_state_server");
  if (argc != 2)
  {
    ROS_INFO("usage: battery_charging_state_server <segway_port>");
    return 1;
  }
  ros::NodeHandle n;
  segway_topic = new std::string(argv[1]);
  /*voltage_reader = new VoltageCurrentSerialInterface();
    sensor_port = new std::string(argv[1]);
    if (voltage_reader->initComm(*sensor_port))
    {
        cout << "Id-mind Port: " << *sensor_port << endl;
        ros::ServiceServer service = n.advertiseService("kokam_battery_charging_state", query_charging_state);
        ROS_INFO("Ready to check Vizzy's charging state.");
        ros::spin();
    }
    else
    {
        ROS_ERROR("Unable to connect to the battery state PCB.");
    }
    voltage_reader->closeComm();*/
  boost::shared_ptr<segway_rmp::SegwayStatusStamped const> segway_status;
  segway_status = ros::topic::waitForMessage<segway_rmp::SegwayStatusStamped>(*segway_topic, ros::Duration(5));
  if (!segway_status)
  {
    ROS_INFO("Segway rmp 50 node is not running/ Segway rmp 50 is off ");
    ROS_INFO("Please check if the segway is turned on and if the segway ros node is running");
    return 0;
  }
  else
  {
    ros::ServiceServer service = n.advertiseService("battery_charging_state", query_segway_charging_state);
    ros::spin();
  }
  return 0;
}
