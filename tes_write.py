#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from datetime import datetime
from std_msgs.msg import Float64, Float32, Int32, Bool
from mavros_msgs.msg import Altitude, State, OverrideRCIn, VFR_HUD, WaypointReached, RCOut

DROP_ZONE_WP_INDEX = 1

LOOP_RATE = 20
CAM_ANGLE = 0

RECORD_POST_EFFECT = True

# Upper & Lower HSV hasil percobaan
LOWER_H = 0
UPPER_H = 22
LOWER_S = 109
UPPER_S = 167
LOWER_V = 250
UPPER_V = 255

# Jumlah vertices dari contour yg ingin dideteksi
VERTICES = 4

# Luas area (dlm pixel) dari kontur
MIN_AREA = 1
MAX_AREA = 2000

if __name__ == '__main__':
  # initialize ros node
  rospy.loginfo('Initializing target_detection.py...')
  rospy.loginfo('Initializing target_detection node...')
  rospy.init_node('target_detection', anonymous=True)

  # initialize target loop rate
  rate = rospy.Rate(LOOP_RATE)

  # initialize video capture
  rospy.loginfo('Initializing camera...')
  cap = cv2.VideoCapture(0)
  frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
  frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

  # initialize video recording
  fourcc = cv2.VideoWriter_fourcc(*'MJPG')
  out = cv2.VideoWriter('/home/ubuntu/' + datetime.now().strftime('%Y-%m-%d_%H:%M:%S') + '.avi', fourcc, LOOP_RATE, (frame_width, frame_height))
  rospy.loginfo(f'Camera initialized, dimensions: {frame_width}x{frame_height}')

  # initializing subscribers
  rospy.loginfo('Initializing subscribers...')

  alt = 0
  def alt_cb(msg):
    global alt
    alt = msg.data
  rospy.Subscriber('/mavros/global_position/rel_alt', Float64, alt_cb)

  fcu_state = State()
  def fcu_state_cb(state):
    global fcu_state
    fcu_state = state
  rospy.Subscriber('/mavros/state', State, fcu_state_cb)
  
  groundspeed = 0
  airspeed = 0
  def vfr_hud_cb(msg):
    global groundspeed, airspeed
    groundspeed = msg.groundspeed
    airspeed = msg.airspeed
  rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, vfr_hud_cb)

  rc_out = RCOut()
  def rc_out_cb(pwm_out):
    global rc_out
    rc_out = pwm_out.channels
  rospy.Subscriber('/mavros/rc/out', RCOut, rc_out_cb)

  # main loop
  rospy.loginfo('object_detection.py initialization complete')

  while not rospy.is_shutdown():
    # retrieve frame
    ret, frame = cap.read()

    if not ret:
      # if camera closed, try reopening camera
      rospy.logerr('Camera unexpectedly closed, trying to reopen camera...')
      cap = cv2.VideoCapture(0)

    if not RECORD_POST_EFFECT:
      # write frame video before drawings
      out.write(frame)

    if RECORD_POST_EFFECT:
      rospy.loginfo_throttle(60, "Writing data")
      # write detection information

    # on every frame
    # write other additional informations
    if fcu_state.connected:
      cur_mode = fcu_state.mode
      fcu_status_color = (0, 255, 0)
    else:
      cur_mode = 'DISCONNECTED'
      fcu_status_color = (0, 0, 255)

    cv2.putText(frame,
                f'MODE: {cur_mode}',
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                fcu_status_color,
                2)
    cv2.putText(frame,
                'ALTITUDE: %.2f' % alt,
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                'GROUNDSPEED: %.2f' % groundspeed,
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                'AIRSPEED: %.2f' % airspeed,
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)
    cv2.putText(frame,
                'OUT RUDDER PWM: %.2f' % rc_out[2],
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)


    out.write(frame)

    # keep the loop at a specified rate
    try:
      rate.sleep()
    except:
      rospy.logwarn('rate.sleep() interrupted by shutdown.')

  rospy.loginfo('Shutting down object_detection.py...')
  rospy.loginfo('Releasing resources...')
  cap.release()
  out.release()
  rospy.loginfo('object_detection.py is shut down')