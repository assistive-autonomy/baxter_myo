import sys, os

import rospy
from std_msgs.msg import String, UInt8
from baxter_interface import Limb, Gripper, Navigator, CHECK_VERSION

from baxter_myo.pose_generator import PoseGenerator


class ArmController(object):

    def __init__(self, starting_poss=None, push_thresh=10,
                 mode='one_arm', arm_mode='first'):
        """
        Initialises parameters and moves the arm to a neutral
        position.
        """
        self.push_thresh = push_thresh
        self._right_neutral_pos = starting_poss[0]
        self._left_neutral_pos = starting_poss[1]
        self._mode = mode
        self._arm_mode = arm_mode

        rospy.loginfo("Creating interface and calibrating gripper")
        self._right_limb = Limb('right')
        self._left_limb = Limb('left')
        self._right_gripper = Gripper('right', CHECK_VERSION)
        self._right_gripper.calibrate()
        if (self._mode == "two_arms"):
            self._left_gripper = Gripper('left', CHECK_VERSION)
            self._left_gripper.calibrate()
        self._is_right_fist_closed = False
        self._is_left_fist_closed = False
        
        # self._right_torso_navigator = Navigator('right_torso')
        self._right_limb_navigator = Navigator('right')

        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        rospy.loginfo("Initialising PoseGenerator")
        self._pg = PoseGenerator(self._mode, self._arm_mode)
        self._sub_right_gesture = rospy.Subscriber("/low_myo/gesture", UInt8,
                                                   self._right_gesture_callback)
        self._sub_left_gesture = rospy.Subscriber("/top_myo/gesture", UInt8,
                                                  self._left_gesture_callback)
        self._last_data = None
        self._right_limb_navigator.outer_led = True
        self._right_limb_navigator.inner_led = True
        self._pg.calibrate()
        self._right_limb_navigator.outer_led = False
        self._right_limb_navigator.inner_led = False

    def move_to_neutral(self):
        if self._mode == "one_arm":
            self._right_limb.move_to_joint_positions(self._right_neutral_pos)
        elif self._mode == "two_arms":
            self._right_limb.move_to_joint_positions(self._right_neutral_pos)
            self._left_limb.move_to_joint_positions(self._left_neutral_pos)
        else:
            raise ValueError("Mode %s is invalid!" % self._mode)

    def is_right_pushing(self):
        """
        Checks if any of the joints is under external stress. Returns
        true if the maximum recorded stress above specified threshold.
        """
        e = self._right_limb.joint_efforts()
        max_effort = max([abs(e[i]) for i in e.keys()])
        return max_effort > self.push_thresh

    def is_left_pushing(self):
        """
        Checks if any of the joints is under external stress. Returns
        true if the maximum recorded stress above specified threshold.
        """
        e = self._left_limb.joint_efforts()
        max_effort = max([abs(e[i]) for i in e.keys()])
        return max_effort > self.push_thresh

    def _command_right_gripper(self):
        """
        Reads state from Myo and opens/closes gripper as needed.
        """

        if not self._right_gripper.ready():
            return
        if self._right_gripper.moving():
            return

        if self._is_right_fist_closed:
            self._right_gripper.close()
        else:
            self._right_gripper.open()

    def _command_left_gripper(self):
        """
        Reads state from Myo and opens/closes gripper as needed.
        """

        if not self._left_gripper.ready():
            return
        if self._left_gripper.moving():
            return
        if self._is_left_fist_closed:
            self._left_gripper.close()
        else:
            self._left_gripper.open()

    def step(self):
        """
        Executes a step of the main routine.
        Fist checks the status of the gripper and
        """
        os.system('clear')
        if self._mode == "one_arm":
            return self.one_arm_step()
        elif self._mode == "two_arms":
            return self.two_arms_step()
        else:
            raise ValueError("Mode %s is invalid!" % self.mode)

    def one_arm_step(self):
        if not self._right_limb_navigator.button0:
            self._command_right_gripper()

            pos = self._pg.generate_pose()

            if pos is not None:
                if not self.is_right_pushing():
                    self._right_limb.set_joint_positions(pos)
                else:
                    rospy.logwarn("Arm is being pushed!")
            else:
                rospy.logwarn("Generated position is invalid")
        else:
            rospy.logwarn("Right Limb Navigator button detected!")
            self._right_limb_navigator.outer_led = True
            self._right_limb_navigator.inner_led = True
            rospy.loginfo("Moving to neutral position")
            self.move_to_neutral()
            rospy.loginfo("Recalibrating PoseGenerator")
            self._pg.calibrate()
            self._right_limb_navigator.outer_led =False
            self._right_limb_navigator.inner_led =False

    def two_arms_step(self):
        self._command_right_gripper()
        self._command_left_gripper()

        poss = self._pg.generate_pose()

        if poss is not None:
            if not self.is_right_pushing():
                self._right_limb.set_joint_positions(poss[0])
            if not self.is_left_pushing():
                self._left_limb.set_joint_positions(poss[1])
            else:
                rospy.logwarn("Arm is being pushed!")
        else:
            rospy.logwarn("Generated position is invalid")

    def _right_gesture_callback(self, data):
        self._is_right_fist_closed = (data.data == 1)

    def _left_gesture_callback(self, data):
        self._is_left_fist_closed = (data.data != 0)


def main():
    ac = ArmController(mode='one_arm')
    while not rospy.is_shutdown():
        ac.step()

if __name__ == "__main__":
    main()
