/***************************************************************************//**
* \file battery_simulator.h
*
* \brief demo of ROS diagnostics for battery (header)
* \author Paul Bouchier
* \date December 28, 2014
*
* Demo for the ROS diagnostics
*
* \section license License (BSD-3)
* Copyright (c) 2014, Paul Bouchier\n
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
* - Neither the name of Willow Garage, Inc. nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef BATTERY_SIMULATOR_H
#define BATTERY_SIMULATOR_H

#include <ros/ros.h>
#include <ros/rate.h>
#include <dynamic_reconfigure/server.h>
#include <diag_demo/BattConfig.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace diag_demo
{

class BattSimulator
{
public:
    BattSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void simulateBatt(double sin_val);
    void config_cb(diag_demo::BattConfig &config, uint32_t level);

private:
    diagnostic_updater::Updater batt_updater_;
    void getBattChargeStatus(
        diagnostic_updater::DiagnosticStatusWrapper& batt_charge_status);

    int batt_sim_delay_;
    double batt_charge_;
    double prev_batt_charge_;
    double batt_warning_charge_;

    dynamic_reconfigure::Server<diag_demo::BattConfig> config_server;
    dynamic_reconfigure::Server<diag_demo::BattConfig>::CallbackType f;
};

BattSimulator::BattSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    batt_sim_delay_(0),
    batt_charge_(100),
    prev_batt_charge_(100),
    batt_warning_charge_(30.0),
    batt_updater_(nh, nh_priv, std::string(" batt_updater"))
{
    batt_updater_.setHardwareID("Battery");
    batt_updater_.add("Battery Charge Status", this, 
                        &BattSimulator::getBattChargeStatus);

   f = boost::bind(&BattSimulator::config_cb, this, _1, _2);
    config_server.setCallback(f);

    std::cout << "Constructed BattSimulator\n";
}

void BattSimulator::simulateBatt(double sin_val)
{
    if (batt_sim_delay_ > 100)
    {
        batt_sim_delay_ = 0;
        batt_charge_ = (sin_val + 1) * 50;
        if (prev_batt_charge_ > 10.0 && batt_charge_ < 10.0)
        {
            ROS_WARN("Battery charge just dropped below 10 percent");
        }
        prev_batt_charge_ = batt_charge_;
        ROS_DEBUG("simulating battery");
    }
    else
    {
        batt_sim_delay_++;
    }

    batt_updater_.update();
}

void BattSimulator::getBattChargeStatus(diagnostic_updater::DiagnosticStatusWrapper& batt_charge_status)
{
    if (batt_charge_ < 10.0)
    {
        batt_charge_status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
                                "Battery charge below 10%" );
    }
    else if (batt_charge_ < batt_warning_charge_)
    {
        batt_charge_status.summary( diagnostic_msgs::DiagnosticStatus::WARN, 
                                "Battery charge below warning level" );
    }
    else
    {
        batt_charge_status.summary( diagnostic_msgs::DiagnosticStatus::OK, 
                                "Battery charge OK" );
    }
    batt_charge_status.add("Battery charge", batt_charge_);
}

void BattSimulator::config_cb(diag_demo::BattConfig &config, uint32_t level)
{
    batt_warning_charge_ = config.battery_warn_level;
    ROS_INFO("Battery Warn level changed to: %lf", config.battery_warn_level);
}

} // namespace

#endif // BATTERY_SIMULATOR
