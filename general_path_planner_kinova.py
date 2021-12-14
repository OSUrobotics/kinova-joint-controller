#! /usr/bin/env python

# Author: Ryan Roberts
# Email: roberyan@oregonstate.edu
# Date: 10/21
# 
# script for capturing/executing joint poses on kinova arm either virtually or in real world.
#
# referenced: kinova_path_planning.py by Nuha Nishat

import rospy
import sys, os, inspect
import time
import numpy as np
import copy
import math
import csv
import rosnode
import functools
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene


class MoveRobot():

    def __init__(self, mode, environment, third_arg, csv_out=None):

        # file paths
        self.main_file_location = os.path.dirname(os.path.abspath(inspect.stack()[0][1]))
        self.joint_angles_dir = self.main_file_location + "/joint_angles/"
        if(not os.path.exists(self.joint_angles_dir)):
            os.makedirs(self.joint_angles_dir)

        try:
            self.mode = int(mode)
        except Exception:
            raise IOError("invalid first argument (must be 0 or 1)")
        #read joint angles
        if(self.mode == 0):
            self.csv_name = self.joint_angles_dir + third_arg
            self.joint_poses = []
            try:
                f = open(self.csv_name, "r")
                f.close()
            except Exception:
                raise IOError("invalid csv file name. Cannot find csv file joint_angles directory")
        #write joint angles
        elif(self.mode == 1):
            self.csv_name = self.joint_angles_dir + csv_out
            try:
                self.run_custom = int(third_arg)
            except Exception:
                raise IOError("invalid second argument (must be 0 or 1)")
            if(self.run_custom < 0 or self.run_custom > 1):
                raise IOError("invalid second argument (must be 0 or 1)")
        else:
            raise IOError("invalid first argument (must be 0 or 1)")
        
        # Initialize moveit commander and ros node for moveit
        try:
            self.environment = int(environment)
        except Exception:
            raise IOError("invalid second argument (must be 0, 1, or 2)")

        # To read from redirected ROS Topic (Gazebo launch use)
        if self.environment == 2:
            joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
            moveit_commander.roscpp_initialize(joint_state_topic)
            rospy.init_node('move_kinova', anonymous=False)
            moveit_commander.roscpp_initialize(sys.argv)
        
        # For real robot launch use
        elif self.environment == 0:
            joint_state_topic = ['joint_states:=/j2s7s300_driver/out/joint_state']
            moveit_commander.roscpp_initialize(joint_state_topic)
            rospy.init_node('move_kinova', anonymous=False)
            moveit_commander.roscpp_initialize(sys.argv)
        
        # for virtual robot launch use
        elif self.environment == 1:
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node('move_kinova', anonymous=True)
        
        else:
            raise IOError("invalid second argument (must be 0, 1, or 2)")

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)

        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)

        #how much to round joint angles
        self.joint_angle_rounded = 2 

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        elif planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        elif planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_goal(self, ee_pose):
        """
        moves robot to current end-effector position

        Input: list of euler or quaternion values for end-effector 
        """
        pose_goal = Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        if len(ee_pose) == 6:
            quat = quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                                                            math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def display_trajectory(self):
        self.disp_pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory,
                                        queue_size=20)
        self.disp.trajectory.append(self.plan)
        print(self.disp.trajectory)
        self.disp_pub.publish(self.disp)

    def go_to_finger_joint_state(self, joint_values):
        """
        moves finger joints to desired joint states.

        Input: list of joint values for each finger joint (list size of 3)

        Output: boolean stating whether path completion was successful (True) or not (False)
        """
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            self.move_gripper.go(wait=True)
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values):
        """
        moves arm joints to desired joint states.

        Input: list of joint values for each arm joint (list size of 7) or name of pre-determined state
        
        current possible state names: (can add more in j2s7s300.srdf)
            - joint_values = "Home"
            - joint_values = "Vertical"

        Output: boolean stating whether path completion was successful (True) or not (False)
        """
        try:
            if(joint_values == "Home"):
                self.move_group.set_named_target("Home")
            elif(joint_values == "Vertical"):
                self.move_group.set_named_target("Vertical")
            else:
                arm_states = JointState()
                arm_states.position = joint_values
                self.move_group.set_joint_value_target(arm_states.position)
            self.move_group.go(wait=True)
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False
    
    def build_env(self, path):
        """
        spawn collision models.
        can be custom or basic meshes
        """
        # see examples
        rospy.loginfo("Spawning collision models for path {}".format(path))

    def teardown_env(self, path):
        """
        despawn collision models.
        """
        # see examples
        rospy.loginfo("Despawning collision models for path {}".format(path))

    def capture_joint_pose(self):
        """
        helper function for write_joint_pose().
        captures current joint values
        """
        self.current_joint_values = self.move_group.get_current_joint_values()
        gripper_values = self.move_gripper.get_current_joint_values()
        for i in range(len(gripper_values)):
            self.current_joint_values.append(gripper_values[i])
        #round joint values
        for i in range(len(self.current_joint_values)):
            self.current_joint_values[i] = round(float(self.current_joint_values[i]), self.joint_angle_rounded)

    def write_joint_pose(self):
        """
        captures current joint values and writes to csv file specified by user.
        """
        self.capture_joint_pose()
        with open(self.csv_name, mode="a") as f:
            writer = csv.writer(f, delimiter=",", quotechar="|")
            writer.writerow(self.current_joint_values)

    def read_joint_poses(self):
        """
        helper function for execute_joint_poses().
        reads joint values from csv and appends to self.joint_poses list
        """
        with open(self.csv_name, mode="r") as f:
            reader = csv.reader(f, delimiter=" ", quotechar="|")
            for row in reader:
                parsed_row = row[0].split(",")
                parsed_row = [float(v) for v in parsed_row]
                self.joint_poses.append(copy.deepcopy(parsed_row))

    def execute_joint_poses(self):
        """
        runs list of joint poses on robot.
        """
        self.read_joint_poses()
        num_poses = len(self.joint_poses)
        for i in range(num_poses):
            rospy.loginfo("Moving to joint pose {} out of {}".format((i + 1), num_poses))
            next_arm_joint_angles = self.joint_poses[i][:7]
            next_gripper_joint_angles = self.joint_poses[i][7:10]
            self.capture_joint_pose()
            current_arm_joint_angles = self.current_joint_values[:7]
            current_gripper_joint_angles = self.current_joint_values[7:10]
            #compare current and next joint lists. If same, don't set new goal, otherwise set and go to new joint goals
            if(not (functools.reduce(lambda x,y : x and y, map(lambda p,q : p == q, current_arm_joint_angles,next_arm_joint_angles), True))):
                self.go_to_arm_joint_state(next_arm_joint_angles)
            if(not (functools.reduce(lambda x,y : x and y, map(lambda p,q : p == q, current_gripper_joint_angles,next_gripper_joint_angles), True))):
                self.go_to_finger_joint_state(next_gripper_joint_angles)
        self.joint_poses = []

    def Run(self):
        """
        main function for executing user commands.
        """

        self.set_planner_type("RRT")

        #run loaded joint angles
        if(self.mode == 0):
            self.execute_joint_poses()
        
        #record joint angles
        elif(self.mode == 1):

            if(self.run_custom):
                # write custom path here
                rospy.loginfo('Running custom path')

            #capture joint poses on command
            else:
                while(True):
                    user_in = raw_input("Enter 0 to record current joint angles. Enter 1 to exit (saves automatically): ")
                    if(user_in == "1"):
                        break
                    elif(user_in == "0"):
                        self.write_joint_pose()
                    else:
                        print("Invalid user input")

if __name__ == '__main__':
    """
    First arg: mode 
        - read joint poses (0)
        - capture joint poses (1)

    Second arg: environment
        - Real robot launch (0)
        - Virtual robot launch (1)
        - redirected ROS topic (i.e. gazebo launch) (2)

    read joint poses:
        third arg: 
            - name of csv file containing joint poses

    capture joint poses:
        third arg: 
            - move robot (in rviz or real world w/ controller) and capture joint angles on command (0)
            - run custom code (1)
        fourth arg:
            - name of csv file to be written to

    Notes:
        - gripper joint values will return as 0.0 until they are actually moved for the first time
        - all joint angle files are read/written from the joint_angles folder
    """
    if(len(sys.argv) == 4):
        controller = MoveRobot(sys.argv[1], sys.argv[2], sys.argv[3])
        controller.Run()
    elif(len(sys.argv) == 5):
        controller = MoveRobot(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
        controller.Run()
    else:
        raise IOError("Invalid number of arguments")
