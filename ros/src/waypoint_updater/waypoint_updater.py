#!/usr/bin/env python
from __future__ import print_function
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight, TrafficLightArray

import math

from dragon_util import DragonUtil
from tf.transformations import euler_from_quaternion
import heapq

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
MAX_ACCEL = 1.
MAX_DECEL = -5.
DROP_RATE = 10


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        # DRAGON:
        self.util = DragonUtil()
        self.base_waypoints = [] # The modified waypoint data for processing
        self.orig_waypoints = [] # The original waypoint messages
        self.last_wp_index = 0
        self.traffic_lights = [] # Latest state of traffic lights
        self.max_accel = None
        self.max_decel = None
        self.pos_count = 0
        self.last_set_speed = 0

        rospy.spin()

    def get_max_decel(self):
        if self.max_decel == None:
            try:
                self.max_decel = rospy.get_param('/dbw_node/decel_limit')
            except KeyError:
                return MAX_DECEL / 5.

        return self.max_decel / 5.

    
    
    def next_light_state(self, nxt_wp_idx, search_range):
        lights = self.traffic_lights
        candidate_idxs = []
        nxt_wp_pos = self.orig_waypoints[nxt_wp_idx].pose.pose.position
        for i in range(len(lights)):
            light_pos = lights[i].pose.pose.position
            straight_dist = self.util.distance(nxt_wp_pos.x, nxt_wp_pos.y, light_pos.x, light_pos.y)
            
            if straight_dist <= search_range:
                light_wp_idx = self.util.closestWaypoint(light_pos.x, light_pos.y, self.base_waypoints)
                light_wp_pos = self.orig_waypoints[light_wp_idx].pose.pose.position
                light_dist = self.util.path_len(nxt_wp_idx, light_wp_idx, self.orig_waypoints, search_range * 2)
                if light_dist == None: light_dist = search_range * 3
                rospy.logerr('Traffic light number %d at x:%d y:%d %dm away snapto:(%d %f %f) pathlen: %f.',
                             i, light_pos.x, light_pos.y, straight_dist, light_wp_idx, light_wp_pos.x,
                             light_wp_pos.y, light_dist)
                if light_dist <= search_range:
                    rospy.logerr('Found traffic light number %d at x:%d y:%d state:%d',
                                 i, light_pos.x, light_pos.y, lights[i].state)
                    heapq.heappush(candidate_idxs, (light_dist, i))
        if len(candidate_idxs):
            light_idx = heapq.heappop(candidate_idxs)[1]
            rospy.logerr('Picked traffic light %d', light_idx)
            return lights[light_idx].state
        else:
            return 4
        

    def pose_cb(self, msg):
#        rospy.loginfo("DRAGON: Received current pose message.")
        # self.pos_count += 1
        # if self.pos_count == DROP_RATE: self.pos_count = 0
        # else: return
        
        euler = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w])
#        rospy.logerr(euler)

        # DRAGON:
        cur_x = msg.pose.position.x
        cur_y = msg.pose.position.y
        nxt_wp_idx = self.util.nextWaypoint(cur_x, cur_y, euler[2], self.base_waypoints, self.last_wp_index)
        if nxt_wp_idx == self.last_wp_index and self.last_set_speed != 0: return
        rospy.logerr("X: %f, Y: %f, wp: %d", cur_x, cur_y, nxt_wp_idx)
        #nxt_wp_idx = self.util.nextWaypoint(cur_x, cur_y, euler[2], self.base_waypoints)

        if nxt_wp_idx < 0 or nxt_wp_idx >= len(self.orig_waypoints):
            rospy.logerr("Cannot find the next waypoint index: %d!", nxt_wp_idx)
            return

        # Now publish the final way points
        lane_msg = Lane()
        last_idx = (nxt_wp_idx+LOOKAHEAD_WPS) % len(self.orig_waypoints)

        # TODO: get actual velocity
        current_vel = self.base_waypoints[nxt_wp_idx]['twist_x']
        max_decel = self.get_max_decel()
        stopin_time = current_vel / max_decel
        stopin_dist = -max_decel * (stopin_time ** 2) / 2.
        
        tl_state = self.next_light_state(nxt_wp_idx, (stopin_dist + 20.))

        for idx in range(LOOKAHEAD_WPS):
            waypoint_idx = (nxt_wp_idx+idx) % len(self.orig_waypoints)
            new_waypoint = Waypoint()
            new_waypoint.pose.pose = self.orig_waypoints[waypoint_idx].pose.pose
            # steering is not ready for such drastic breaking yet but at least it doesn't break completely
            target_speed = 0 if tl_state in (0, 1) else self.base_waypoints[waypoint_idx]['twist_x']
            #target_speed = self.base_waypoints[waypoint_idx]['twist_x']
            new_waypoint.twist.twist.linear.x = target_speed
            lane_msg.waypoints.append(new_waypoint)
        self.last_set_speed = target_speed


        if tl_state != 4:
            rospy.logerr('Next traffic light state: %d', tl_state)
        # rospy.logerr("Publishing lane message starting from: %d, %d",
        #              self.orig_waypoints[nxt_wp_idx].pose.pose.position.x,
        #              self.orig_waypoints[nxt_wp_idx].pose.pose.position.y)
        # rospy.logerr("Difference b/w cur and way point: %d, %d", self.orig_waypoints[nxt_wp_idx].pose.pose.position.x - cur_x,
        #              self.orig_waypoints[nxt_wp_idx].pose.pose.position.y - cur_y)
        self.final_waypoints_pub.publish(lane_msg)

        # Store the last waypoint index to narrow down the search
        self.last_wp_index = nxt_wp_idx

    def waypoints_cb(self, waypoints):
        rospy.loginfo("DRAGON: Received the base waypoints messages")
        #rospy.logerr(waypoints)


        # DRAGON: Store it in local variable, ignore header information for now

        # DRAGON: Fill up the necessay information from received message


        for waypoint in waypoints.waypoints:
            #rospy.logerr(waypoint)
            euler = euler_from_quaternion([waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y,
                                           waypoint.pose.pose.orientation.z, waypoint.pose.pose.orientation.w])
            #rospy.logerr("Euler: %s", str(euler))

            self.base_waypoints.append({'x': waypoint.pose.pose.position.x, 'y':waypoint.pose.pose.position.y,
                                        'angle':euler[2],'twist_x': waypoint.twist.twist.linear.x})
            self.orig_waypoints.append(waypoint)
        #for wp in self.base_waypoints:
        total_way_points = len(self.base_waypoints)
        prev_s_dist = 0
        for index in range(total_way_points):
            wp = self.base_waypoints[index]

            next_wp_index = index
            if next_wp_index == total_way_points:
                next_wp_index = 0
            #fn = util.getFrenet(wp['x'], wp['y'], wp['angle'], self.base_waypoints, next_wp_index, prev_s_dist)
            fn = self.util.getFrenet(wp['x'], wp['y'], wp['angle'], self.base_waypoints, next_wp_index, prev_s_dist)
            prev_s_dist = fn['s']
#            rospy.logerr(wp)
#            rospy.logerr(fn)
            #pass
        rospy.logerr("All done")




    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # DRAGON: For now this gets the lights from simulator
        self.traffic_lights = msg.lights

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
