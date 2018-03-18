#include "diag_demo/gps_battery_analyzer.h"
#include "std_msgs/String.h"


#include <iostream>
#include <string>
#include <vector>

using namespace diagnostic_aggregator;

PLUGINLIB_REGISTER_CLASS(GpsBatteryAnalyzer,  
                         diagnostic_aggregator::GpsBatteryAnalyzer, 
                         diagnostic_aggregator::Analyzer)

GpsBatteryAnalyzer::GpsBatteryAnalyzer() :
    path_("GPS_Battery_Analyzer"),
    nice_name_("GPS_pluginAnalyzer"),
    gps_name_(""),
    has_initialized_(false),
    has_battery_data_(false),
    has_gps_data_(false)
{
}

GpsBatteryAnalyzer::~GpsBatteryAnalyzer() {}

bool GpsBatteryAnalyzer::init(const std::string base_name, const ros::NodeHandle &n)
{
    // This is an example of getting a parameter from the analyzer yaml file
    if (!n.getParam("gps_name", gps_name_))
    {
        ROS_ERROR("No gps name was specified in GpsBatteryAnalyzer! GPS must be \"GPS\". Namespace: %s", n.getNamespace().c_str());
        return false;
    }

    // Make a "missing" item for the GPS
    boost::shared_ptr<StatusItem> item(new StatusItem(path_));
    gps_item_ = item;

    ros::NodeHandle nh;
    gps_battery_pub_ = nh.advertise<std_msgs::String>("gps_battery", 10);
    has_initialized_ = true;
  
  return true;
}

bool GpsBatteryAnalyzer::match(const std::string name)
{
    if (name == "batt_updater: Battery Charge Status") {
        return true;
    }
    return name == gps_name_;
}

bool GpsBatteryAnalyzer::analyze(const boost::shared_ptr<StatusItem> item)
{
    // This analyzer doesn't necessarily do anything oriented toward a goal - it
    // models looking at battery & gps data & producing a cross-function
    // analysis of the health of a gps receiver's battery
    //
    // It is an example of how to build an analyzer, modeled after
    // http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer

    if (item->getName() == "batt_updater: Battery Charge Status")
    {
        has_battery_data_ = true;
        gps_item_ = item;
        battery_charge_ = atoi(item->getValue("Battery charge").c_str());
        return false; // Won't report this item
    }

    // We know our item is "GPS"
    has_gps_data_ = true;

    return true;
}

std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > GpsBatteryAnalyzer::report()
{
    boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> gps_stat = gps_item_->toStatusMsg("/GPS");
    gps_stat->name = path_;
    gps_stat->message = "GPS battery plugin analysis";
    gps_stat->hardware_id = "GPS battery";

    // If we have battery data, and battery is low, we'll suppress gps errors
  if (has_battery_data_ && battery_charge_ < 50)
  {
    gps_stat->level = diagnostic_msgs::DiagnosticStatus::WARN;
  }
  else
  {
    gps_stat->level = diagnostic_msgs::DiagnosticStatus::OK;
  }


  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > output;
  output.push_back(gps_stat);

  return output;
}


