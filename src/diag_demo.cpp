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
    x_(0.0),
    sin_val_(0.0),
    gps_sim_(nh, nh_priv),
    batt_sim_(nh, nh_priv)
{
    std::cout << "Constructed DiagDemo\n";
}

void DiagDemo::generateHwSimData()
{
    while (ros::ok())
    {
        sin_val_ = sin(x_);
        x_ += 0.002;

        gps_sim_.simulateGPS(sin_val_);
        batt_sim_.simulateBatt(sin_val_);

        hw_sim_rate_.sleep();
    }
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

