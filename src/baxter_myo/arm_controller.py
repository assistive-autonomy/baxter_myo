import sys, os

import rospy
from std_msgs.msg import String, UInt8
from baxter_interface import Limb, Gripper, Navigator, DigitalIO, CHECK_VERSION

from baxter_myo.pose_generator import PoseGenerator

class InputHandle(object):
    def __init__(self, hold_t=1.0):
        self._right_torso_navigator = Navigator('torso_right')
        self._right_limb_navigator = Navigator('right')
        self._right_shoulder_button = DigitalIO('right_shoulder_button')

        self.hold_threshold = rospy.Duration.from_sec(hold_t)
        
        # State variables
        self._right_torso_state = self._right_torso_navigator.button0
        self._righ_limb_state = self._right_limb_navigator.button0
        self._right_torso_last_state = self._right_torso_navigator.button0
        self._righ_limb_last_state = self._right_limb_navigator.button0

        self.press_time = 0.0

        # Internal State tracking
        self._states = {
            'torso': {'curr': False, 'last': False, 'start': rospy.Duration(0)},
            'limb':  {'curr': False, 'last': False, 'start': rospy.Duration(0)},
            'shoulder': {'curr': 1,     'last': 1,     'start': rospy.Duration(0)}
        }
        
        # Event storage
        self._events = {
            'torso': {'pressed': False, 'holding': False, 'released': False},
            'limb':  {'pressed': False, 'holding': False, 'released': False},
            'shoulder': {'pressed': False, 'holding': False, 'released': False}
        }

    @property
    def torso_pressed(self):
        return self._events['torso']['pressed']

    @property
    def torso_holding(self):
        return self._events['torso']['holding']

    @property
    def torso_released(self):
        return self._events['torso']['released']

    @property
    def limb_pressed(self):
        return self._events['limb']['pressed']

    @property
    def limb_holding(self):
        return self._events['limb']['holding']

    @property
    def limb_released(self):
        return self._events['limb']['released']
    
    @property
    def shoulder_pressed(self):
        return self._events['shoulder']['pressed']

    @property
    def shoulder_holding(self):
        return self._events['shoulder']['holding']

    @property
    def shoulder_released(self):
        return self._events['shoulder']['released']

    def update(self):
        now = rospy.Time.now()

        self._states['torso']['curr'] = self._right_torso_navigator.button0
        self._states['limb']['curr']  = self._right_limb_navigator.button0
        self._states['shoulder']['curr'] = self._right_shoulder_button.state

      
        self._process('torso', True, now)
        self._process('limb',  True, now)
        self._process('shoulder', 0,    now)

        for key in self._states:
            self._states[key]['last'] = self._states[key]['curr']

    def _process(self, key, pressed_val, now):
        s = self._states[key]
        e = self._events[key]
        
        being_pressed = (s['curr'] == pressed_val)
        was_pressed = (s['last'] == pressed_val)

        # Reset impulsive flags
        e['pressed'] = False
        e['released'] = False

        if being_pressed and not was_pressed:
            e['pressed'] = True
            s['start'] = now
        elif being_pressed and was_pressed:
            e['holding'] = (now - s['start']) >= self.hold_threshold
        elif not being_pressed and was_pressed:
            e['released'] = True
            e['holding'] = False
            s['start'] = 0.0
    
    def blink(self):
        for i in range(3):    
            self.leds_off()
            self.leds_on()

    def leds_on(self): 
        self._right_limb_navigator.outer_led = True
        self._right_limb_navigator.inner_led = True
        self._right_torso_navigator.outer_led = True
        self._right_torso_navigator.inner_led = True
        rospy.sleep(0.1)
        
    def leds_off(self):   
        self._right_limb_navigator.outer_led = False
        self._right_limb_navigator.inner_led = False
        self._right_torso_navigator.outer_led = False
        self._right_torso_navigator.inner_led = False
        rospy.sleep(0.1)        

        


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
        

        self.baxter_input = InputHandle()
        # self._right_torso_navigator = Navigator('torso_right')
        # self._right_limb_navigator = Navigator('right')
        # self._right_shoulder_button = DigitalIO('right_shoulder_button')
        # self.pause = False

        rospy.loginfo("Moving to neutral position")
        self.move_to_neutral()
        rospy.loginfo("Initialising PoseGenerator")
        self._pg = PoseGenerator(self._mode, self._arm_mode, self.baxter_input)
        self._sub_right_gesture = rospy.Subscriber("/low_myo/gesture", UInt8,
                                                   self._right_gesture_callback)
        self._sub_left_gesture = rospy.Subscriber("/top_myo/gesture", UInt8,
                                                  self._left_gesture_callback)
        self._last_data = None
        self.baxter_input.update()
        self.baxter_input.leds_on()
        self._pg.calibrate()
        self.baxter_input.leds_off()

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
        self.baxter_input.update()
        if self._mode == "one_arm":
            return self.one_arm_step()
        elif self._mode == "two_arms":
            return self.two_arms_step()
        else:
            raise ValueError("Mode %s is invalid!" % self.mode)

    def one_arm_step(self):
        
        ## torso and arm to recalibrate
        if (self.baxter_input.limb_pressed):
            rospy.logwarn("Right Navigator button detected!")
            self.baxter_input.leds_on()
            rospy.loginfo("Moving to neutral position")
            self.move_to_neutral()
            rospy.loginfo("Recalibrating PoseGenerator")
            self.baxter_input.leds_on()
            self._pg.calibrate()
            self.baxter_input.leds_off()
            return
        ## pause with right shoulder button held
        
        if self.baxter_input.shoulder_holding:
            rospy.logwarn("Shoulder button detected! PAUSING")
            rospy.sleep(rospy.Duration(1))
            self.baxter_input.leds_on()
        else:
            rospy.logwarn("Shoulder button not detected! RESUMING")
            rospy.sleep
            self.baxter_input.leds_off()

            self._command_right_gripper()
            pos = self._pg.generate_pose()

            if pos is not None:
                if not self.is_right_pushing():
                    self._right_limb.set_joint_positions(pos)
                else:
                    rospy.logwarn("Arm is being pushed!")
            else:
                rospy.logwarn("Generated position is invalid")

            

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
