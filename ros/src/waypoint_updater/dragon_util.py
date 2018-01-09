from __future__ import print_function
import math
import rospy


class DragonUtil(object):
    def __init__(self):
        pass


    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    def forward_count(self, start, end, length):
        return end - start if start <= end else length - (start - end)

    def path_len(self, start, end, waypoints):
        waypoint_count = len(waypoints)
        dot_count = self.forward_count(start, end, waypoint_count)
        result = 0.
        for i in range(dot_count):
            current = waypoints[(start + i) % waypoint_count].pose.pose.position
            next_point = waypoints[(start + i + 1) % waypoint_count].pose.pose.position
            result += self.distance(current.x, current.y, next_point.x, next_point.y)
        return result
    
    def closestWaypoint(self, x, y, waypoints, start_idx=0, lookup_range=None):
        closestLen = 100000 #large number
        closest_waypoint_index = -1
        total_waypoints = len(waypoints)


        if start_idx < 0 or start_idx >= total_waypoints:
            start_idx = 0
            lookup_range = None # find in whole range

        if start_idx > 100:
            start_idx -= 100
        else:
            start_idx = 0
            lookup_range = None

        if lookup_range is not None:
            end_idx = start_idx + lookup_range # Look in only next 100 waypoints, to speed up things
        else:
            end_idx = total_waypoints

        if end_idx > total_waypoints:
            end_idx = total_waypoints
            start_idx = 0

        for index in range(start_idx, end_idx ):
            wp = waypoints[index]
            wp_x = wp["x"]
            wp_y = wp["y"]
            dist = self.distance(x, y, wp_x, wp_y)
            if dist < closestLen:
                closestLen = dist
                closest_waypoint_index = index
        return closest_waypoint_index

    def nextWaypoint(self, x, y, theta, waypoints, start_idx=0):

        closest_waypoint_index = self.closestWaypoint(x,y, waypoints, start_idx=start_idx, lookup_range=200)
        #closest_waypoint_index = self.closestWaypoint(x, y, waypoints, start_idx=start_idx)

        if closest_waypoint_index == -1:
            assert(False)
            return -1

        closest_waypoint = waypoints[closest_waypoint_index]
        cwp_x = closest_waypoint["x"]
        cwp_y = closest_waypoint["y"]

        heading = math.atan2((cwp_y-y),(cwp_x-x))

        angle = math.fabs(theta-heading)
        angle = min(2*math.pi - angle, angle)

        if angle > math.pi/4:
            closest_waypoint_index += 1
            if closest_waypoint_index == len(waypoints):
                closest_waypoint_index = 0


        return closest_waypoint_index

    def getFrenet(self, x, y, theta, waypoints, next_way_point_index=None, prev_s_dist=None):

        if next_way_point_index is None:
        #if 1:
            next_wp_index = self.nextWaypoint(x, y, theta, waypoints)
            #rospy.logerr("Next_wp_index: %d and passed index: %d", next_wp_index, next_way_point_index)
        else:
            next_wp_index = next_way_point_index

        prev_wp_index = next_wp_index - 1

        # Assuming circular track
        if next_wp_index == 0:
            prev_wp_index  = len(waypoints)-1


        n_x = waypoints[next_wp_index]["x"] - waypoints[prev_wp_index]["x"]
        n_y = waypoints[next_wp_index]["y"] - waypoints[prev_wp_index]["y"]
        x_x = x - waypoints[prev_wp_index]["x"]
        x_y = y - waypoints[prev_wp_index]["y"]

        rospy.logerr("n_x: %f n_y: %f, x_x: %f, x_y: %f", n_x, n_y, x_x, x_y)

        # find the projection of x onto n
        proj_norm = (x_x * n_x+x_y * n_y) / (n_x * n_x+n_y * n_y)
        proj_x = proj_norm * n_x
        proj_y = proj_norm * n_y
        rospy.logerr("proj_norm: %f proj_x: %f, proj_y: %f", proj_norm, proj_x, proj_y)

        frenet_d = self.distance(x_x, x_y, proj_x, proj_y)
        rospy.logerr("frenet_d: %f ", frenet_d)

        # see if d value is positive or negative by comparing it to a center point
        #
        # Is the center at 1000,2000 ?? according to calculations they are correct
        #

        center_x = 1000-waypoints[prev_wp_index]["x"]
        center_y = 2000-waypoints[prev_wp_index]["y"]
        centerToPos = self.distance(center_x, center_y, x_x, x_y)
        centerToRef = self.distance(center_x, center_y, proj_x, proj_y)

        if centerToPos <= centerToRef:
            frenet_d *= -1


        # calculate s value
        frenet_s = 0
        if prev_s_dist is None:
        #if 1:

            for i in range(prev_wp_index):
                frenet_s += self.distance(waypoints[i]["x"], waypoints[i]["y"], waypoints[i+1]["x"], waypoints[i+1]["y"])
            #rospy.logerr("Calculated prev distance: %d and passed distance: %d", frenet_s, prev_s_dist)

        else:
            frenet_s = prev_s_dist

        cur_dist = self.distance(0, 0, proj_x, proj_y)
        frenet_s += cur_dist
        #rospy.logerr("Cur distance: %f frenet_s: %f", cur_dist, frenet_s)

        return {'s': frenet_s, 'd': frenet_d}
