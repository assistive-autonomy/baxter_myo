#!/usr/bin/python
import time
import rospy
import cv_bridge
import cv2
import rospkg

from baxter_myo.arm_controller import ArmController
from baxter_myo.config_reader import ConfigReader

from sensor_msgs.msg import (
    Image
)


def send_image():
        """
        Send the image located at the specified path to the head
        display on Baxter.
        @param path: path to the image file to load and send
        """
        rp = rospkg.RosPack()
        path = rp.get_path('baxter_myo') \
               + '/share/' + 'pink_face.jpg'
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=10)
        pub.publish(msg)
        rospy.sleep(1)

def main():
    rospy.init_node("baxter_myo_controller")
    #c = ConfigReader("one_arm_extended")
    left = {'left_e0': -2.9816751530456544, 'left_e1': 1.062665189593506, 'left_s0': -0.7083156279968262, 'left_s1': 1.0492428577148438, 'left_w0': 1.5888205992370608, 'left_w1': 0.02070874061279297, 'left_w2': 3.049937298028565}
    right = {'right_e0': 1.6010924473554007, 'right_e1': -0.04295146206079158, 'right_s0': 0.8379370053824072, 'right_s1': -0.044485442848677, 'right_w0': -1.593806038612945, 'right_w1': 0.016490293469768196, 'right_w2': -0.03834951969713534}
    push_thresh = 28
    mode = 'one_arm'
    arm_mode = 'first'
    #c.parse_all()

    # s = ArmController((c.right_angles, c.left_angles), c.push_thresh, c.mode, c.arm_mode)
    s = ArmController((right, left), push_thresh, mode, arm_mode)
    #send_image()
    while not rospy.is_shutdown():
        s.step()

if __name__ == "__main__":
    main()
