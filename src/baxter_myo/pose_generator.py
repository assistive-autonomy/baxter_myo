import os
from math import radians, degrees, fabs
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from baxter_interface import Limb, Navigator


class PoseGenerator(object):

    def __init__(self, mode, arm_mode, baxter_input, step=5):
        self.mode = mode
        self.arm_mode = arm_mode
        self._step = step
        self._right_limb = Limb('right')
        self._left_limb = Limb('left')
        self._right_torso_navigator = Navigator('torso_right')
        self._right_limb_navigator = Navigator('right')
        self.baxter_input = baxter_input
        self._subscribe()

    def _subscribe(self):
        self._last_data_0 = (Vector3(), Vector3())
        self._last_data_1 = (Vector3(), Vector3())
        self._calib_data_0 = (Vector3(), Vector3())
        self._calib_data_1 = (Vector3(), Vector3())
        self._sub_ori_0 = rospy.Subscriber("/low_myo/imu_rpy",
                                           Vector3,
                                           self._orientation_callback_0)
        self._sub_pos_0 = rospy.Subscriber("/low_myo/imu_pos",
                                           Vector3,
                                           self._position_callback_0)
        self._sub_ori_1 = rospy.Subscriber("/top_myo/imu_rpy",
                                           Vector3,
                                           self._orientation_callback_1)
        self._sub_pos_1 = rospy.Subscriber("/top_myo/imu_pos",
                                           Vector3,
                                           self._position_callback_1)

    def _orientation_callback_0(self, data):
        self._last_data_0[0].x = data.x
        self._last_data_0[0].y = data.y
        self._last_data_0[0].z = data.z

    def _position_callback_0(self, data):
        self._last_data_0[1].x = data.x
        self._last_data_0[1].y = data.y
        self._last_data_0[1].z = data.z

    def _orientation_callback_1(self, data):
        self._last_data_1[0].x = data.x
        self._last_data_1[0].y = data.y
        self._last_data_1[0].z = data.z

    def _position_callback_1(self, data):
        self._last_data_1[1].x = data.x
        self._last_data_1[1].y = data.y
        self._last_data_1[1].z = data.z

    def calibrate(self):
        """
        Calibrate position of the robot arm wrt the myo data
        """
        self.baxter_input.blink()
        self.baxter_input.leds_on('pause')
        while not rospy.is_shutdown():
            self.baxter_input.update()
            if (self.baxter_input.torso_top_pressed or self.baxter_input.limb_top_pressed):
                rospy.logwarn("Navigator Top button detected!")
                self._calib_data_0 = deepcopy(self._last_data_0)
                self._calib_data_1 = deepcopy(self._last_data_1)
                self._right_calib_pose = self._right_limb.joint_angles()
                self._left_calib_pose = self._left_limb.joint_angles()
                break
        self.baxter_input.leds_off()

    def _is_vector_valid(self, data):
        """
        Check if data is likely to be valid.
        """
        test = data.x + data.y + data.z
        return test != 0

    def _is_over_step(self, change):
        return abs(change) > self._step

    def generate_pose(self):
        """
        Given new data and position of the arm, calculate new
        joint positions.
        """

        if self.mode == "one_arm":
            return self.one_arm_generate_pose()
        elif self.mode == "two_arms":
            return self.two_arms_generate_pose()
        else:
            raise ValueError("Mode %s is invalid!" % self.mode)

    def one_arm_generate_pose(self):
        rospy.logdebug("Generating pose")

        data_0 = deepcopy(self._last_data_0[0])
        data_1 = deepcopy(self._last_data_1[0])
        this_pose = deepcopy(self._right_calib_pose)

        if self.arm_mode == "first":
            if self._is_vector_valid(data_1):
                change_1 = Vector3()
                change_1.x = data_1.x - self._calib_data_1[0].x
                change_1.y = data_1.y - self._calib_data_1[0].y
                change_1.z = data_1.z - self._calib_data_1[0].z
                if (change_1.x < -179):
                    change_1.x += 360
                if (change_1.y < -179):
                    change_1.y += 360
                if (change_1.z < -179):
                    change_1.z += 360
                if (change_1.x > 179):
                    change_1.x -= 360
                if (change_1.y > 179):
                    change_1.y -= 360
                if (change_1.z > 179):
                    change_1.z -= 360
                # print "MYO_1 (Upper arm)"
                rospy.logdebug("Data_1.x %f" % data_1.x)
                rospy.logdebug("CalibData_1.x %f" % self._calib_data_1[0].x)
                rospy.logdebug("Data_1.y %f" % data_1.y)
                rospy.logdebug("CalibData_1.y %f" % self._calib_data_1[0].y)
                rospy.logdebug("Data_1.z %f" % data_1.z)
                rospy.logdebug("CalibData_1.z %f" % self._calib_data_1[0].z)
                # print "ROLL: ", change_1.z
                if (self._is_over_step(change_1.z)):
                    this_pose["right_e0"] += radians(-1 * change_1.z)
                # print "PITCH: ", change_1.y
                if (self._is_over_step(change_1.y)):
                    # this_pose["right_e1"] += radians(-1 * change_1.y)
                    this_pose["right_s1"] += radians(1 * change_1.y)
                # print "YAW: ", change_1.x
                if (self._is_over_step(change_1.x)):
                    this_pose["right_s0"] += radians(-1 * change_1.x)

            if self._is_vector_valid(data_0):
                change_0 = Vector3()
                change_0.x = data_0.x - self._calib_data_0[0].x - change_1.x
                change_0.y = data_0.y - self._calib_data_0[0].y - change_1.y
                change_0.z = data_0.z - self._calib_data_0[0].z - change_1.z
                if (change_0.x < -179):
                    change_0.x += 360
                if (change_0.y < -179):
                    change_0.y += 360
                if (change_0.z < -179):
                    change_0.z += 360
                if (change_0.x > 179):
                    change_0.x -= 360
                if (change_0.y > 179):
                    change_0.y -= 360
                if (change_0.z > 179):
                    change_0.z -= 360
                # print "MYO_0 (Forearm)"
                rospy.logdebug("Data_0.x %f" % data_0.x)
                rospy.logdebug("CalibData_0.x %f" % self._calib_data_0[0].x)
                rospy.logdebug("Data_0.y %f" % data_0.y)
                rospy.logdebug("CalibData_0.y %f" % self._calib_data_0[0].y)
                rospy.logdebug("Data_0.z %f" % data_0.z)
                rospy.logdebug("CalibData_0.z %f" % self._calib_data_0[0].z)
                # print "ROLL: ", change_0.z
                if (self._is_over_step(change_0.z)):
                    this_pose["right_w0"] += radians(-1 * change_0.z)
                    # this_pose["right_w0"] += radians(1 * change_0.z)
                # print "PITCH: ", change_0.y
                if (self._is_over_step(change_0.y)):
                    # this_pose["right_w1"] += radians(-1 * change_0.y)
                    # this_pose["right_w1"] += radians(-1 * change_0.y)
                    this_pose["right_w1"] += radians(1 * change_0.y)
                # print "YAW: ", change_0.x
                if (self._is_over_step(change_0.x)):
                    # this_pose["right_w2"] += radians(-1 * change_0.x)
                    # this_pose["right_w2"] += radians(1 * change_0.x)
                    this_pose["right_e1"] += radians(-1 * change_0.x)

        elif self.arm_mode == "second":
            print "SECOND!"

            # Alex you need to change below
            if self._is_vector_valid(data_1):
                change_1 = Vector3()
                change_1.x = data_1.x - self._calib_data_1[0].x
                change_1.y = data_1.y - self._calib_data_1[0].y
                change_1.z = data_1.z - self._calib_data_1[0].z
                print "MYO_1 (Upper arm)"
                print "ROLL: ", change_1.x
                if (self._is_over_step(change_1.x)):
                    this_pose["right_e0"] += radians(change_1.x)
                print "PITCH: ", data_1.y
                if (self._is_over_step(change_1.y)):
                    this_pose["right_s1"] += radians(-1 * change_1.y)
                print "YAW: ", change_1.z
                if (self._is_over_step(change_1.z)):
                    this_pose["right_s0"] += radians(change_1.z)
            if self._is_vector_valid(data_0):
                change_0 = Vector3()
                change_0.x = data_0.x - self._calib_data_0[0].x
                change_0.y = data_0.y - self._calib_data_0[0].y
                change_0.z = data_0.z - self._calib_data_0[0].z
                print "MYO_0 (Forearm)"
                print "PITCH: ", data_0.y
                print "ROLL: ", data_0.x
                if (self._is_over_step(change_0.x)):
                    this_pose["right_w2"] += radians(-1 * change_0.x)
                if (self._is_over_step(change_0.y)):
                    change_0.y += change_1.y
                    this_pose["right_e1"] += radians(-1 * change_0.y)
                print "YAW: ", data_0.z
                if (self._is_over_step(change_0.z)):
                    this_pose["right_w1"] += radians(change_0.z)

        return this_pose

    def two_arms_generate_pose(self):
        rospy.logdebug("Generating pose")

        data_0 = deepcopy(self._last_data_0[0])
        data_1 = deepcopy(self._last_data_1[0])
        right_pose = deepcopy(self._right_calib_pose)
        left_pose = deepcopy(self._left_calib_pose)

        if self.arm_mode == "first":
            if self._is_vector_valid(data_0):
                change_0 = Vector3()
                change_0.x = data_0.x - self._calib_data_0[0].x
                change_0.y = data_0.y - self._calib_data_0[0].y
                change_0.z = data_0.z - self._calib_data_0[0].z
                print "MYO_0 (Right forearm)"
                print "PITCH: ", change_0.y
                if (self._is_over_step(change_0.y)):
                    right_pose["right_w1"] += radians(-1 * change_0.y)
                print "YAW: ", change_0.z
                if (self._is_over_step(change_0.z)):
                    right_pose["right_w0"] += radians(-1 * change_0.z)
                print "ROLL: ", change_0.x
                if (self._is_over_step(change_0.x)):
                    right_pose["right_w2"] += radians(-1 * change_0.x)

            if self._is_vector_valid(data_1):
                change_1 = Vector3()
                change_1.x = data_1.x - self._calib_data_1[0].x
                change_1.y = data_1.y - self._calib_data_1[0].y
                change_1.z = data_1.z - self._calib_data_1[0].z
                print "MYO_1 (Left forearm)"
                print "PITCH: ", change_1.y
                if (self._is_over_step(change_1.y)):
                    left_pose["left_w1"] += radians(-1 * change_1.y)
                print "YAW: ", change_1.z
                if (self._is_over_step(change_1.z)):
                    left_pose["left_w0"] += radians(-1 * change_1.z)
                print "ROLL: ", change_1.x
                if (self._is_over_step(change_1.x)):
                    left_pose["left_w2"] += radians(-1 * change_1.x)

        elif self.arm_mode == "second":
            if self._is_vector_valid(data_0):
                change_0 = Vector3()
                change_0.x = data_0.x - self._calib_data_0[0].x
                change_0.y = data_0.y - self._calib_data_0[0].y
                change_0.z = data_0.z - self._calib_data_0[0].z
                print "MYO_0 (Right forearm)"
                print "PITCH: ", change_0.y
                if (self._is_over_step(change_0.y)):
                    right_pose["right_e1"] += radians(change_0.y)
                print "YAW: ", change_0.z
                if (self._is_over_step(change_0.z)):
                    right_pose["right_w1"] += radians(change_0.z)
                print "ROLL: ", change_0.x
                if (self._is_over_step(change_0.x)):
                    right_pose["right_w2"] += radians(-1 * change_0.x)

            if self._is_vector_valid(data_1):
                change_1 = Vector3()
                change_1.x = data_1.x - self._calib_data_1[0].x
                change_1.y = data_1.y - self._calib_data_1[0].y
                change_1.z = data_1.z - self._calib_data_1[0].z
                print "MYO_1 (Left forearm)"
                print "PITCH: ", change_1.y
                if (self._is_over_step(change_1.y)):
                    left_pose["left_e1"] += radians(-1 * change_1.y)
                print "YAW: ", change_1.z
                if (self._is_over_step(change_1.z)):
                    left_pose["left_w1"] += radians(-1 * change_1.z)
                print "ROLL: ", change_1.x
                if (self._is_over_step(change_1.x)):
                    left_pose["left_w2"] += radians(change_1.x)

        return (right_pose, left_pose)
