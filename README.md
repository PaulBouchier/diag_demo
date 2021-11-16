# Diag_demo package

This package provides a demo of the ROS diagnostic framework.

### diag_demo package block diagram
![diag_demo block diagram](/Doc/diag_demo.drawio.png)

## Running the demo

Launch the demo with roslaunch diag_demo diag_demo.launch. This launches rqt_robot_monitor alongside the diagnostic nodes. You can launch just the diag_demo nodes without robot monitor using diag_demo_noviz.launch. In this case, rosrun rqt_robot_monitor rqt_robot_monitor to see the visualization.