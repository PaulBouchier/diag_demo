/***************************************************************************//**
* \file diag_demo.h
*
* \brief demo of ROS diagnostics (header)
* \author Paul Bouchier
* \date December 27, 2014
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

#ifndef DIAG_DEMO_H
#define DIAG_DEMO_H

#include "diag_demo/gps_simulator.h"

#include <ros/ros.h>
#include <ros/rate.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/update_functions.h>

#include <boost/thread.hpp>

namespace diag_demo
{

class DiagDemo
{
public:
    DiagDemo(ros::NodeHandle nh, ros::NodeHandle nh_priv);
    void generateHwSimData();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    ros::Rate hw_sim_rate_;

    double x_;
    double sin_val_;
    GPSSimulator gps_sim_;
};


} // namespace

#endif // DIAG_DEMO_H
