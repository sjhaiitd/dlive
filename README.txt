Information shared by Rohit Mohan (BIT Mesra) rohitmohan1914@gmail.com

Steps to run Car Codes:
1.rosrun zed_wrapper zed_wrapper_node
2.rosrun vo vo_code.py
3.roslaunch driverless_car planning.launch (runs navigation stack, nav_can.py, controller)

4.change scale of pid in rqt_reconfigure(till this problem is solved)
5.change in auto_mode
6. either use rviz or run gui.
   rosrun driverless_car gui.py  

GPS_client.py receives data from the topic and plots on map
client1.py receives data from server and publishes stereo odom data on a topic

Instructions for plotting:

GPS_client.py is the main file which does the plotting work.
Have OpenCV installed along with OpenGL 

Ensure GPS socket address is same on the transmission side as well as in GPS_client.py.
Publish the odometry data on the vo_pos topic which is of string format. 

Run GPS_Client.py for real-time plotting

x_vo and y_vo are plotted on the map. Data received on vo_pos topic is processed to obtain x_vo and y_vo. 

In case of plotting data from any other source, write a suitable function for processing the data return it in x_vo and y_vo.


Using OpenStreetMap (openstreetmap.org) I generated a .osm file which is a xml file containing lat long information of a given area. (Using export feature of site).

This xml file contains all the information regarding roads, buildings etc in tag form. I went through ogm_cartography package of ros. The whole package wasn't required for our aim of generating static binary map for
global planner. I wrote my own script to generate a .jpg file and .yaml file for a given .osm file using necessary functions from osm_cartography package. 

use - roslaunch driverless_car planning.launch  for planning part
use- rosrun driverless_car map_gen.py for map generation

[July 12, 2017]
Today's doubt solved one of my problems that I was facing.

Question was whether the planner take velocity data or not and the answer is yes it takes.

I edited the plugin which generated the odom data relative to gazebo world. I made all linear and angular velocity zero. In this case it showed the same symptoms that I had mentioned before.

The speed of in twist msg of cmd_vel never exceeded 0.5m/s (given by move_base). And then I realized that when the car is moving in x direction, base_link and odom have same direction so speed went to 5m/s or more. When base_link was at 90degree to odom. Odom.twist.linear.x became zero and Odom.twist.linear.y became base_link velocity.

The local planner took odom.twist.linear.x as input which was zero and behaved in the same 0.5m/s speed. Since for the planner, feedback told it that car is till at 0 so it never gave high speed input to follow.
I changed the plugin in such a way that now odom reflects required condition. And Teb_local_planner works fine in terms of speed. Oscillation is still there which needs to be addressed.

Summary: Odom should contain the x velcoity which is the base_link velocity for proper working.
and angular velocity as well in z direction.

Note: Base_link is the car';s main link wrt to which all other parameters such sensors are placed.
