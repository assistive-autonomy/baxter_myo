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

        # Internal State tracking
        self._states = {
            'torso_top': {'curr': False, 'last': False, 'start': rospy.Duration(0)},
            'torso_bottom': {'curr': False, 'last': False, 'start': rospy.Duration(0)},            
            'limb_top':  {'curr': False, 'last': False, 'start': rospy.Duration(0)},
            'limb_bottom': {'curr': False, 'last': False, 'start': rospy.Duration(0)},
            'shoulder': {'curr': 1,     'last': 1,     'start': rospy.Duration(0)}
        }
        
        # Event storage
        self._events = {
            'torso_top': {'pressed': False, 'holding': False, 'released': False},
            'torso_bottom': {'pressed': False, 'holding': False, 'released': False},
            'limb_top':  {'pressed': False, 'holding': False, 'released': False},
            'limb_bottom':  {'pressed': False, 'holding': False, 'released': False},            
            'shoulder': {'pressed': False, 'holding': False, 'released': False}
        }

    @property
    def torso_top_pressed(self):
        return self._events['torso_top']['pressed']

    @property
    def torso_top_holding(self):
        return self._events['torso_top']['holding']

    @property
    def torso_top_released(self):
        return self._events['torso_top']['released']

    @property
    def torso_bottom_pressed(self):
        return self._events['torso_bottom']['pressed']

    @property
    def torso_bottom_holding(self):
        return self._events['torso_bottom']['holding']

    @property
    def torso_bottom_released(self):
        return self._events['torso_bottom']['released']

    @property
    def limb_top_pressed(self):
        return self._events['limb_top']['pressed']

    @property
    def limb_top_holding(self):
        return self._events['limb_top']['holding']

    @property
    def limb_top_released(self):
        return self._events['limb_top']['released']

    @property
    def limb_bottom_pressed(self):
        return self._events['limb_bottom']['pressed']

    @property
    def limb_bottom_holding(self):
        return self._events['limb_bottom']['holding']

    @property
    def limb_bottom_released(self):
        return self._events['limb_bottom']['released']
    
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

        self._states['torso_top']['curr'] = self._right_torso_navigator.button1
        self._states['torso_bottom']['curr'] = self._right_torso_navigator.button2
        self._states['limb_top']['curr']  = self._right_limb_navigator.button1
        self._states['limb_bottom']['curr']  = self._right_limb_navigator.button2
        self._states['shoulder']['curr'] = self._right_shoulder_button.state

        self._process('torso_top', True, now)
        self._process('torso_bottom', True, now)
        self._process('limb_top',  True, now)
        self._process('limb_bottom',  True, now)        
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

    def leds_on(self, state=None):
        if state is None:
            self._right_limb_navigator.outer_led = True
            self._right_limb_navigator.inner_led = True
            self._right_torso_navigator.outer_led = True
            self._right_torso_navigator.inner_led = True
        elif state == 'pause':
            self._right_limb_navigator.outer_led = False
            self._right_limb_navigator.inner_led = True
            self._right_torso_navigator.outer_led = False
            self._right_torso_navigator.inner_led = True
        elif state == 'auto':
            self._right_limb_navigator.outer_led = True
            self._right_limb_navigator.inner_led = False
            self._right_torso_navigator.outer_led = True
            self._right_torso_navigator.inner_led = False
        else:
            rospy.logwarn("Invalid LED state. Please use 'pause' or 'auto'")
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
        self.is_paused = False

        self.baxter_input.leds_on('auto')
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
        self.baxter_input.leds_on('pause')
        self._pg.calibrate()
        self.baxter_input.leds_on()

    def move_to_neutral(self):
        if self._mode == "one_arm":
            self._right_limb.move_to_joint_positions(self._right_neutral_pos, threshold=0.1)
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
        if rospy.is_shutdown():
            self.baxter_input.leds_off()

    def one_arm_step(self):
        
        ## torso and arm to recalibrate
        if (self.baxter_input.torso_top_pressed or self.baxter_input.limb_top_pressed):
            rospy.logwarn("Navigator Top button detected!")
            self.baxter_input.leds_on('auto')
            rospy.loginfo("Moving to neutral position")
            self.move_to_neutral()
            rospy.loginfo("Recalibrating PoseGenerator")
            self.baxter_input.leds_on('pause')
            self._pg.calibrate()
            self.baxter_input.leds_on()
            self.is_paused = False
            return

        ## pause/unpause with navigator bottom buttons
        if (self.baxter_input.torso_bottom_pressed or self.baxter_input.limb_bottom_pressed):
            if not self.is_paused:
                rospy.logwarn("Navigator Top button detected! PAUSING")
                self.baxter_input.leds_on('pause')
                self.is_paused = True
            else:
                rospy.logwarn("Navigator Top button detected! RESUMING")
                self.baxter_input.leds_on()
                self.is_paused = False
        elif self.is_paused:
            rospy.logwarn_throttle(5, "Demo currently PAUSED!")
        else:
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
