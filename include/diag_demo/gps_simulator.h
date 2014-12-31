/***************************************************************************//**
* \file gps_simulator.h
*
* \brief demo of ROS diagnostics for gps (header)
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

#ifndef GPS_SIMULATOR_H
#define GPS_SIMULATOR_H

#include <ros/ros.h>
#include <ros/rate.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

namespace diag_demo
{

class GPSSimulator
{
public:
    GPSSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void simulateGPS(double sin_val);

private:
    diagnostic_updater::Updater gps_updater_;   //!< Update at rate set by parameter 
    diagnostic_updater::FrequencyStatus gps_freq_status_;
    void getGPSDiagStatus(diagnostic_updater::DiagnosticStatusWrapper& gps_diag_status);
    double min_gps_rate_;
    double max_gps_rate_;
    int gps_sim_delay_;
    int gps_pkts_sent_;
    int gps_pkts_dropped_;
    int num_sats_;
};

GPSSimulator::GPSSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    min_gps_rate_(0.5),
    max_gps_rate_(10),
    gps_sim_delay_(0),
    gps_pkts_sent_(0),
    gps_pkts_dropped_(0),
    num_sats_(0),
    gps_updater_(nh, nh_priv, std::string("ggps_updater")),
    gps_freq_status_(diagnostic_updater::FrequencyStatusParam(&min_gps_rate_, &max_gps_rate_, 0.1, 10))
{
    gps_updater_.setHardwareID("GPS");
    gps_updater_.add("GPS Diag Status", this, &GPSSimulator::getGPSDiagStatus);
    gps_updater_.add(gps_freq_status_); // add the frequency reporting task
}

void GPSSimulator::simulateGPS(double sin_val)
{
    if (gps_sim_delay_ > 20)
    {
        gps_sim_delay_ = 0;

        if (sin_val > 0)
        {
            // diagnostics for publishing gps data with occasional dropouts
            gps_pkts_sent_++;
            gps_freq_status_.tick();
        }
        else
        {
            gps_pkts_dropped_++;
        }
        // Set number of satellites to vary between 0 & 10
        num_sats_ = static_cast<int>((sin_val + 1) * 5);
    }
    else
    {
        gps_sim_delay_++;
    }

    gps_updater_.update();
}

void GPSSimulator::getGPSDiagStatus(diagnostic_updater::DiagnosticStatusWrapper& gps_diag_status)
{
    if (num_sats_ > 4)
    {
        gps_diag_status.summary( diagnostic_msgs::DiagnosticStatus::OK, 
                                "PIKSI status OK" );
    }
    else
    {
        gps_diag_status.summary( diagnostic_msgs::DiagnosticStatus::ERROR, 
                                "Insufficient satellites for rtk fix" );
    }
    gps_diag_status.add("Packets received", gps_pkts_sent_);
    gps_diag_status.add("Packets dropped", gps_pkts_dropped_);
}


} // namespace

#endif // GPS_SIMULATOR
