#!/usr/bin/python3

import rospy
import time
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import SetMode

LOOP_RATE = 1
DROP_MECHANISM_CHANNEL = 6
DROP_MECHANISM_PWMS = [1500, 1250, 1750]

if __name__ == '__main__':
    # initialize ros node
    rospy.init_node('dropping_mechanism_test', anonymous=True)

    # initialize target loop rate
    rate = rospy.Rate(LOOP_RATE)

    # initializing client
    rospy.loginfo('Initializing client...')
    mode = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    rospy.wait_for_service("/mavros/set_mode")
    try:
        mode
        mode1 = mode(custom_mode = "MANUAL")
        while not mode1.mode_sent:
            rospy.loginfo_throttle(3600, "Change Mode to MANUAL")
            mode1 = mode(custom_mode = "MANUAL")
        rospy.loginfo_once("MANUAL FLIGHT MODE")
    except rospy.ServiceException as e:
        rospy.logerr(e)

    # initializing publishers
    rospy.loginfo('Initializing publishers...')
    override_rc_in_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

    # center the servo
    override_rc_in_msg = OverrideRCIn()
    override_rc_in_msg.channels[DROP_MECHANISM_CHANNEL] = DROP_MECHANISM_PWMS[0]
    override_rc_in_pub.publish(override_rc_in_msg)
    time.sleep(5)

    # main loop
    rospy.loginfo('dropping_mechanism_test.py initialization complete')
    cur_state_index = 0
    while not rospy.is_shutdown():
        override_rc_in_msg = OverrideRCIn()
        override_rc_in_msg.channels[DROP_MECHANISM_CHANNEL] = DROP_MECHANISM_PWMS[cur_state_index]
        override_rc_in_pub.publish(override_rc_in_msg)
        cur_state_index = (cur_state_index + 1) % len(DROP_MECHANISM_PWMS)

        time.sleep(5)

    rospy.loginfo('Shutting down dropping_mechanism_test.py...')
    rospy.loginfo('dropping_mechanism_test.py is shut down')