#ifndef GPS_BATTERY_ANALYZER_H
#define GPS_BATTERY_ANALYZER_H

#include <ros/ros.h>
#include <diagnostic_aggregator/analyzer.h>
#include <diagnostic_aggregator/status_item.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <pluginlib/class_list_macros.h>

namespace diagnostic_aggregator {

class GpsBatteryAnalyzer : public Analyzer
{
public:
  GpsBatteryAnalyzer();
  ~GpsBatteryAnalyzer();

  bool init(const std::string base_name, const ros::NodeHandle &n);

  bool match(const std::string name);

  bool analyze(const boost::shared_ptr<StatusItem> item);

  std::vector<boost::shared_ptr<diagnostic_msgs::DiagnosticStatus> > report();

  std::string getPath() const { return path_; }

  std::string getName() const { return nice_name_; }

private:
  ros::Publisher gps_battery_pub_;

  // Store status item for battery
  boost::shared_ptr<StatusItem> gps_item_;

  std::string path_;
  std::string nice_name_;
  std::string gps_name_;
  bool has_initialized_;
  bool has_battery_data_;
  bool has_gps_data_;
  int battery_charge_;
};

} // namespace diagnostic_aggregator

#endif // GPS_BATTERY_ANALYZER_H
