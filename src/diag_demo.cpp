/***************************************************************************//**
* \file diag_demo.cpp
*
* \brief demo of ROS diagnostics
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

#include "diag_demo/diag_demo.h"


#include <cstdlib>

namespace diag_demo
{


DiagDemo::DiagDemo(ros::NodeHandle nh, ros::NodeHandle nh_priv) :
    nh_(nh),
    nh_priv_(nh_priv),
    hw_sim_rate_(100),
    min_gps_rate_(0.5),
    max_gps_rate_(10),
    gps_sim_delay_(0),
    gps_pkts_sent_(0),
    gps_pkts_dropped(0),
    num_sats_(0),
    hw_sim_time_(0),
    x_(0.0),
    sin_val_(0.0),
    gps_freq_status_(diagnostic_updater::FrequencyStatusParam(&min_gps_rate_, &max_gps_rate_, 0.1, 10))
{
    gps_updater_.setHardwareID("GPS");
    gps_updater_.add("GPS Diag Status", this, &DiagDemo::getGPSDiagStatus);
    gps_updater_.add(gps_freq_status_); // add the frequency reporting task

}

void DiagDemo::generateHwSimData()
{
    while (ros::ok())
    {
        sin_val_ = sin(x_);
        x_ += 0.002;

        hw_sim_rate_.sleep();
    }
}

void DiagDemo::simulateGPS()
{
    if (gps_sim_delay_ > 20)
    {
        gps_sim_delay_ = 0;

        if (sin_val_ > 0)
        {
            // diagnostics for publishing gps data with occasional dropouts
            std::cout << "Emitting GPS data\n";
            gps_pkts_sent++;
            gps_freq_status_.tick();
        }
        else
        {
            gps_pkts_dropped++;
        }
        // Set number of satellites to vary between 0 & 10
        num_sats_ = static_cast<int>((sin_val_ + 1) * 5);
    }
    else
    {
        gps_sim_delay_++;
    }

    gps_updater_.update();
}

void DiagDemo::getGPSDiagStatus(diagnostic_updater::DiagnosticStatusWrapper& gps_diag_status )
{
    
}


}   // namespace

/*! \brief Main Function
* Initializes ROS, instantiates the diagnostics emitters
* and the diagnostic updaters
* \param argc Number of command line arguments
* \param argv 2D character array of command line arguments
* \return EXIT_SUCCESS, or an error state
*/
int main( int argc, char *argv[] )
{
	ros::init( argc, argv, "diag_demo" );

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv( "~" );

    diag_demo::DiagDemo diag_demo(nh, nh_priv);
    diag_demo.generateHwSimData();

    return 0;
}

