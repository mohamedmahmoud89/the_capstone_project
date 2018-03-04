#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import tf.transformations as tf
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 300 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wps=[]
	self.is_wps_available = False
	self.is_pos_update_available = False
	self.max_speed = 4.4 # mps = ~10mph
	self.min_speed = 2.2 # mps = ~5mph 
	self.speed = 0. # start at standstill
        self.schedule(1)

    def schedule(self,time):	
	while rospy.is_shutdown() != True:
	    if self.is_wps_available == True and self.is_pos_update_available == True:
	    	final_wps = self.wps_update()
		self.publish(final_wps)
	    rospy.sleep(time)

    def update_velocity(self,fwps):
	veh_yaw = tf.euler_from_quaternion([self.pos.orientation.x,
					  self.pos.orientation.y,
					  self.pos.orientation.z,
					  self.pos.orientation.w])
	wp_yaw = tf.euler_from_quaternion([fwps[0].pose.pose.orientation.x,
					  fwps[0].pose.pose.orientation.y,
					  fwps[0].pose.pose.orientation.z,
					  fwps[0].pose.pose.orientation.w])
	
	#TODO: use trignometrics instead of assuming always vehicle angle = 45° for better linearization
	if math.fabs(self.pos.position.y - fwps[0].pose.pose.position.y) <= 1. and math.fabs(veh_yaw[2] - wp_yaw[2]) <= np.pi/40.:
	    self.speed += .44
	else:
	    self.speed -= .44
	
	if self.speed > self.max_speed:
	    self.speed = self.max_speed
	if self.speed < self.min_speed:
	    self.speed = self.min_speed

	for wp in fwps:
	    wp.twist.twist.linear.x = self.speed
	    wp.twist.twist.linear.y = 0.
	    wp.twist.twist.linear.z = 0.
	    wp.twist.twist.angular.x = 0.
	    wp.twist.twist.angular.y = 0.
	    wp.twist.twist.angular.z = 0.

	print("speed = " + str(fwps[0].twist.twist.linear.x))

	return fwps

    def wps_update(self):
        current_idx = 0
        final_wps = []
	is_point_located = False
        for idx in range(len(self.base_wps)):
            if (self.pos.position.x > self.base_wps[idx].pose.pose.position.x and
		self.pos.position.x < self.base_wps[(idx+1)%len(self.base_wps)].pose.pose.position.x):
		temp_x = self.pos.position.x + math.fabs(self.pos.position.y - self.base_wps[idx + 1].pose.pose.position.y)
		is_point_located = True

	    if(is_point_located == True and
	       temp_x > self.base_wps[idx].pose.pose.position.x and
	       temp_x < self.base_wps[(idx+1)%len(self.base_wps)].pose.pose.position.x):
		current_idx = idx + 1
                break
        for idx in range(0,LOOKAHEAD_WPS):
            final_wps.append(self.base_wps[(idx + current_idx)%len(self.base_wps)])

	if len(final_wps) > 0:
	    final_wps = self.update_velocity(final_wps)
	return final_wps

    def pose_cb(self, msg):
        # TODO: Implement
	self.is_pos_update_available = True
        self.pos = msg.pose

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints.waypoints
	self.is_wps_available = True
       
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish(self,waypoints):
        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
