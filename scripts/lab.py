#!/usr/bin/env python3

import copy
import rospy
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import yaml
from geometry_msgs.msg import Pose, Point, PointStamped
from spar_lab_control.srv import *
from std_srvs.srv import Trigger, TriggerResponse
import rospkg
from sofia_perception.srv import DetectObject
import rosbag


class Lab(object):
    def __init__(self):
        self.x_min = rospy.get_param('/x_min')
        self.x_max = rospy.get_param('/x_max')
        self.y_min = rospy.get_param('/y_min')
        self.y_max = rospy.get_param('/y_max')
        self.z_min = rospy.get_param('/z_min')
        self.z_max = rospy.get_param('/z_max')

        self.base_frame = rospy.get_param('/base_frame')
        self.camera_frame = rospy.get_param('/camera_frame')

        self.pc_topic = '/camera/depth_registered/points'

        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('spar_lab_control') + '/resources/'


        print('Waiting for services')
        rospy.wait_for_service('go_to_joint_goal') # ovaj servis advertisea control
        rospy.wait_for_service('go_to_pose_goal') # ovaj servis advertisea control
        rospy.wait_for_service('go_to_position_goal') # ovaj servis advertisea control
        rospy.wait_for_service('detect_object') # ovaj servis advertiseaju studenti

        self.test_publisher = rospy.Publisher(
            'test_pc', PointCloud2, queue_size=10)
        self.point_publisher = rospy.Publisher(
            'test_point', PointStamped, queue_size=10)

        self.go_to_joint_goal_client = rospy.ServiceProxy(
            'go_to_joint_goal', jointGoal)
        self.go_to_pose_goal_client = rospy.ServiceProxy(
            'go_to_pose_goal', poseGoal)
        self.go_to_position_goal_client = rospy.ServiceProxy(
            'go_to_position_goal', positionGoal)
        self.detect_objects_client = rospy.ServiceProxy(
            'detect_object', DetectObject)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        print('Sleeping for 3s - tf subscriber initialized')
        rospy.sleep(3)

        rospy.Service('mission_done', Trigger, self.mission_done_callback)
        self.mission_done = False

        self.bag = rosbag.Bag('perception.bag', 'w')

    def mission_done_callback(self, req):
        self.mission_done = True
        return TriggerResponse()

    def check_mission_done(self):
        while not rospy.is_shutdown():
            if self.mission_done:
                self.mission_done = False
                return

    def read_vector_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        return(dict.get('positions')[0][0])

    def read_vectors_from_yaml(self, file):
        joint_goals = []
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)

        l = len(dict.get('positions'))
        for i in range(l):
            joint_goals.append(dict.get('positions')[i][0])

        return joint_goals

    def read_pose_from_yaml(self, file):
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)
        vector = dict.get('positions')[0][0]
        pose_goal = Pose()
        pose_goal.position.x = vector[0]
        pose_goal.position.y = vector[1]
        pose_goal.position.z = vector[2]
        pose_goal.orientation.x = vector[3]
        pose_goal.orientation.y = vector[4]
        pose_goal.orientation.z = vector[5]
        pose_goal.orientation.w = vector[6]
        return pose_goal

    def read_poses_from_yaml(self, file):
        poses_goal = []
        path = self.yaml_path + file
        with open(path) as f:
            dict = yaml.safe_load(f)

        l = len(dict.get('positions'))
        for i in range(l):
            vector = dict.get('positions')[i][0]
            pose_goal = Pose()
            pose_goal.position.x = vector[0]
            pose_goal.position.y = vector[1]
            pose_goal.position.z = vector[2]
            pose_goal.orientation.x = vector[3]
            pose_goal.orientation.y = vector[4]
            pose_goal.orientation.z = vector[5]
            pose_goal.orientation.w = vector[6]
            poses_goal.append(pose_goal)

        return poses_goal

    def record(self, joint_goal):
        self.go_to_joint_goal_client.call(joint_goal)
        self.check_mission_done()

        print('Waiting for pc message')
        self.pc_msg = rospy.wait_for_message(self.pc_topic, PointCloud2)
        print('Recieved pc message')

        print('Waiting for tf')
        try:
            trans = self.tfBuffer.lookup_transform(
                self.base_frame, self.camera_frame, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('Did not get transform')

        self.pc_glob = do_transform_cloud(self.pc_msg, trans)
        #self.bag.write('/pc_glob', self.pc_glob)
        #self.bag.close()
        # while(True):
        #     self.test_publisher.publish(self.pc_glob)
        #     rospy.sleep(3)
        print('Transformed point cloud')

    def check_limits(self, c):
        if c.x < self.x_min or c.x > self.x_max or \
           c.y < self.y_min or c.y > self.y_max or \
           c.z < self.z_min or c.z > self.z_max:
            return False

        return True

    def task1(self):
        place_pt = Point()
        place_pt.x = 0.215
        place_pt.y = -0.35
        place_pt.z = 0.2

        approach_place_pt = copy.deepcopy(place_pt)
        approach_place_pt.z += 0.1

        req = positionGoalRequest()

        detected_object = self.detect_objects_client.call(self.pc_glob)
        centroid_pt = detected_object.object_centroid
        print("Object centroid: ")
        print(centroid_pt)
        if not self.check_limits(centroid_pt):
            print("Detected centroid out of limits!")
            return

        # approach_pt = copy.deepcopy(centroid_pt)
        # approach_pt.z += 0.33
        # grasp_pt = copy.deepcopy(centroid_pt)
        # grasp_pt.z += 0.24

        # req.position = approach_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved to approach pose")

        # req.position = grasp_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved to grasp pose")

        # print("Ready to grasp!")
        # rospy.sleep(10)

        # req.position = approach_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved above object")

        # self.go_home()

        # req.position = approach_place_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved above place location")

        # req.position = place_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved to place pose")

        # print("Ready to release!")
        # rospy.sleep(10)

        # req.position = approach_place_pt
        # self.go_to_position_goal_client.call(req)
        # self.check_mission_done()
        # print("Moved above place location")

        # self.go_home()

        # print("Done with pick and place task.")

    def go_home(self):
        joint_goal = self.read_vector_from_yaml(
            'kinova_home.yaml')
        self.go_to_joint_goal_client.call(joint_goal)
        self.check_mission_done()
        print("Moved home")


def main():
    rospy.init_node('laboratory_exercise')
    lab = Lab()
    # lab.go_home()

    joint_goal = lab.read_vector_from_yaml(
        'kinova_record.yaml')
    lab.record(joint_goal)
    print("Done with recording")

    # lab.go_home()

    print("Starting laboratory exercise - pick and place task")
    lab.task1()


if __name__ == "__main__":
    main()
