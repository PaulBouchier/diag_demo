/***************************************************************************//**
* \file camera_simulator.h
*
* \brief demo of ROS diagnostics for camera (header)
* \author Paul Bouchier
* \date December 31, 2014
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

#ifndef CAMERA_SIMULATOR_H
#define CAMERA_SIMULATOR_H

#include <ros/ros.h>
#include <ros/rate.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>

namespace diag_demo
{

class CameraSimulator
{
public:
    CameraSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void simulateCamera(double sin_val);

private:
    diagnostic_updater::Updater camera_updater_;   //!< Update at rate set by parameter 
    diagnostic_updater::FrequencyStatus camera_freq_status_;
    void getCameraDiagStatus(diagnostic_updater::DiagnosticStatusWrapper& camera_diag_status);
    double min_camera_rate_;
    double max_camera_rate_;
    int camera_sim_delay_;
    int camera_pkts_sent_;
    int camera_pkts_dropped_;
    int num_sats_;
};

CameraSimulator::CameraSimulator(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    min_camera_rate_(29),
    max_camera_rate_(31),
    camera_sim_delay_(0),
    camera_pkts_sent_(0),
    camera_pkts_dropped_(0),
    camera_updater_(nh, nh_priv, std::string("gcamera_updater")),
    camera_freq_status_(diagnostic_updater::FrequencyStatusParam(&min_camera_rate_, &max_camera_rate_, 0.1, 10))
{
    camera_updater_.setHardwareID("Camera");
    camera_updater_.add("Camera Diag Status", this, &CameraSimulator::getCameraDiagStatus);
    camera_updater_.add(camera_freq_status_); // add the frequency reporting task
}

void CameraSimulator::simulateCamera(double sin_val)
{
    if (camera_sim_delay_ > 1)
    {
        camera_sim_delay_ = 0;

        if (sin_val < 0.99)
        {
            // diagnostics for publishing camera data with occasional dropouts
            camera_pkts_sent_++;
            camera_freq_status_.tick();
        }
        else
        {
            camera_pkts_dropped_++;
        }
    }
    else
    {
        camera_sim_delay_++;
    }

    camera_updater_.update();
}

void CameraSimulator::getCameraDiagStatus(diagnostic_updater::DiagnosticStatusWrapper& camera_diag_status)
{
    camera_diag_status.summary( diagnostic_msgs::DiagnosticStatus::OK, 
                            "Camera status OK" );
    camera_diag_status.add("Packets received", camera_pkts_sent_);
    camera_diag_status.add("Packets dropped", camera_pkts_dropped_);
}


} // namespace

#endif // CAMERA_SIMULATOR
