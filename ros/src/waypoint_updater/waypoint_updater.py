#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # DRAGON: Lets store the current x and y coordinates for now
        self.cur_x = 0.0
        self.cur_y = 0.0

        # DRAGON: The base waypoints. We receive them once from waypoint_loader node
        self.base_waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        rospy.loginfo("DRAGON: Received current pose message.")
        #rospy.logerr(msg)

        # DRAGON: Update the current x and y coordinates, we can ignore other values for now
        self.cur_x = msg.pose.position.x
        self.cur_y = msg.pose.position.y
        rospy.logerr("X: %f, Y: %f", self.cur_x, self.cur_y)
        rospy.logerr(self.cur_y)

    def waypoints_cb(self, waypoints):
        rospy.loginfo("DRAGON: Received the base waypoints messages")

        # DRAGON: Store it in local variable, ignore header information for now
        self.base_waypoints = waypoints.waypoints


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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
