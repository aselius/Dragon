#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
from tf.transformations import euler_from_quaternion
import tensorflow as TF
import cv2
import yaml
import numpy as np

import PIL.Image

import sys
sys.path.append('../waypoint_updater')
from dragon_util import DragonUtil

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        rospy.logerr(TF.__version__)

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # Load the tensorflow model
        self.util = DragonUtil()
        self.base_waypoints = []  # The modified waypoint data for processing
        self.last_wp_index = 0

        self.process_cnt = 5 # Process only every 5th image

        import os
        cwd = os.getcwd()
        rospy.logerr("__Working directory__:%s", cwd)

        PATH_TO_CKPT = '../../../classifier/data' + '/udacity_frozen_inference_graph.pb'
        # List of the strings that is used to add correct label for each box.
        PATH_TO_LABELS = '../../../classifier/data' + 'udacity_label_map.pbtxt'

        NUM_CLASSES = 4  # 90

        #print(PATH_TO_CKPT)

        self.detection_graph = TF.Graph()
        with self.detection_graph.as_default():
            od_graph_def = TF.GraphDef()
            with TF.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                TF.import_graph_def(od_graph_def, name='')



        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

        euler = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y,
                                       msg.pose.orientation.z, msg.pose.orientation.w])
        #        rospy.logerr(euler)

        # DRAGON:
        cur_x = msg.pose.position.x
        cur_y = msg.pose.position.y
        nxt_wp_idx = self.util.nextWaypoint(cur_x, cur_y, euler[2], self.base_waypoints, self.last_wp_index)
        rospy.logerr("X: %f, Y: %f, wp: %d", cur_x, cur_y, nxt_wp_idx)
        # nxt_wp_idx = self.util.nextWaypoint(cur_x, cur_y, euler[2], self.base_waypoints)

        if nxt_wp_idx < 0 or nxt_wp_idx >= len(self.base_waypoints):
            rospy.logerr("Cannot find the next waypoint index: %d!", nxt_wp_idx)
            return

        self.last_wp_index = nxt_wp_idx

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

        for waypoint in waypoints.waypoints:
            #rospy.logerr(waypoint)
            euler = euler_from_quaternion([waypoint.pose.pose.orientation.x, waypoint.pose.pose.orientation.y,
                                           waypoint.pose.pose.orientation.z, waypoint.pose.pose.orientation.w])
            #rospy.logerr("Euler: %s", str(euler))

            self.base_waypoints.append({'x': waypoint.pose.pose.position.x, 'y':waypoint.pose.pose.position.y,
                                        'angle':euler[2],'twist_x': waypoint.twist.twist.linear.x})

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
            rospy.logerr(wp)
            rospy.logerr(fn)
            #pass
        rospy.logerr("All done")


    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        if self.process_cnt > 0:
            self.process_cnt -= 1
            # Publish before returning
            self.upcoming_red_light_pub.publish(self.last_wp)
            return

        else:
            self.process_cnt = 5



        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        rospy.logerr("*****************Got image********************8")

        image_np = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        #image = PIL.Image.fromarray(image_np)

        rospy.logerr("Conversion successful")

        with self.detection_graph.as_default():
            with TF.Session(graph=self.detection_graph) as sess:
                # Definite input and output Tensors for detection_graph
                image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')


                #image = Image.open(image_path)
                # the array based representation of the image will be used later in order to prepare the
                # result image with boxes and labels on it.
                #image_np = load_image_into_numpy_array(image)
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                # Actual detection.
                (boxes, scores, classes, num) = sess.run(
                    [detection_boxes, detection_scores, detection_classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                # Visualization of the results of a detection.

                if (int)(np.squeeze(classes)[0]) == 2 and np.squeeze(scores)[0] > 0.4:
                    rospy.logerr("Red detected with accuracy %f", np.squeeze(scores)[0])
                    self.last_wp = self.last_wp_index # I am using last_wp_index to narrow down the search and last_wp is used to publish
                    self.upcoming_red_light_pub.publish(self.last_wp)
                else:
                    self.last_wp = -1
                    self.upcoming_red_light_pub.publish(self.last_wp)

                #print('classes=', np.squeeze(classes)[0])
                #print('scores=', np.squeeze(scores)[0])


        return # Ignore the code below for now
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        return 0

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
