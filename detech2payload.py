#!/usr/bin/python3

import rospy
import cv2
import math
import numpy as np
from datetime import datetime
from std_msgs.msg import Float64, Float32, Int32, Bool
from mavros_msgs.msg import Altitude, State, OverrideRCIn, VFR_HUD, WaypointReached, RCOut
from sensor_msgs.msg import Imu

from improc.contour_detection import get_contour
from improc.get_object_distance import calc_ground_dist, calc_ground_dist_2, calc_obj_min_max_area

#DROP_ZONE_WP_INDEX = 99
DROP_ZONE_START_WP = 2
DROP_ZONE_START_WP2 = 2
DROP_ZONE_END_WP = 4
DROP_ZONE_END_WP2 = 4

APPROACH_DROPPING_WP_INDEX = DROP_ZONE_START_WP

LOOP_RATE = 30
ENCODED_FPS = 30

# sudut penempatan kamera
CAM_ANGLE = 40

RECORD_POST_EFFECT = True

# elevasi drop zone terhadap gcs
DROP_ZONE_ELEVATION = 30

# Lebar drop zone asli dlm meter
DROP_ZONE_REAL_WIDTH = 4.7

# Upper & Lower HSV hasil percobaan
LOWER_H = 0
LOWER_S = 113
LOWER_V = 53

UPPER_H = 19
UPPER_S = 255
UPPER_V = 255

# Jumlah vertices dari contour yg ingin dideteksi
VERTICES = 4

# Luas area (dlm pixel) dari kontur
MIN_AREA = 500
MAX_AREA = 3000

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
  #cap = cv2.VideoCapture('/home/ubuntu/video_tes_1.mp4')
  frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
  frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

  center_of_frame = [frame_width/2, frame_height/2]

  # initialize video recording
  fourcc = cv2.VideoWriter_fourcc(*'PIM1')
  out = cv2.VideoWriter('/home/ubuntu/' + datetime.now().strftime('%Y-%m-%d_%H:%M:%S') + '.avi', fourcc, ENCODED_FPS, (frame_width, frame_height))
  rospy.loginfo(f'Camera initialized, dimensions: {frame_width}x{frame_height}')

  # initializing subscribers
  rospy.loginfo('Initializing subscribers...')

  alt = 0
  def alt_cb(msg):
    global alt
    alt = msg.data
  rospy.Subscriber('/mavros/global_position/rel_alt', Float64, alt_cb)

  rudder_out_pwm = 0
  def rudder_out_pwm_cb(pwm_out):
    global rudder_out_pwm
    rudder_out_pwm = pwm_out.channels[3]
  rospy.Subscriber('/mavros/rc/out', RCOut, rudder_out_pwm_cb)

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

  wp_reached = 0
  def wp_reached_cb(msg):
    global wp_reached
    wp_reached = msg.wp_seq
  rospy.Subscriber('/mavros/mission/reached', WaypointReached, wp_reached_cb)

  pitch = 0
  def attitude_cb(msg):
    global pitch
    q = msg.orientation
    pitch = math.degrees(math.asin(-2.0*(q.x*q.z - q.w*q.y)))
  rospy.Subscriber('/mavros/imu/data', Imu, attitude_cb)

  is_payload_dropped = False
  def is_payload_dropped_cb(msg):
    global is_payload_dropped
    is_payload_dropped = msg.data
  rospy.Subscriber('/fiachra/control/is_payload_dropped', Bool, is_payload_dropped_cb)

  is_target_aligned = False
  def is_target_aligned_cb(msg):
    global is_target_aligned
    is_target_aligned = msg.data
  rospy.Subscriber('/fiachra/control/is_aligned', Bool, is_target_aligned_cb)

  drop_travel_dist = 999
  def drop_travel_dist_cb(msg):
    global drop_travel_dist
    drop_travel_dist = msg.data
  rospy.Subscriber('/fiachra/control/drop_travel_dist', Float32, drop_travel_dist_cb)

  # initialize publishers
  rospy.loginfo('Initializing publishers...')

  target_dist_pub = rospy.Publisher('/fiachra/vision/target_dist', Float32, queue_size=10)
  target_dist_msg = Float32()
  target_dist_msg.data = 999
  target_dist_pub.publish(target_dist_msg)
  
  # range 0 - 1 (with 0 being left edge and 1 being right edge)
  target_x_pub = rospy.Publisher('/fiachra/vision/target_x', Float32, queue_size=10)  
  target_x_msg = Float32()
  target_x_msg.data = 0
  target_x_pub.publish(target_x_msg)

  is_target_detected_pub = rospy.Publisher('/fiachra/vision/is_target_detected', Bool, queue_size=10)
  # predefined values
  is_target_detected_true = Bool()
  is_target_detected_true.data = True
  is_target_detected_false = Bool()
  is_target_detected_false.data = False
  is_target_detected_pub.publish(is_target_detected_false)
  # initialize default is_target_detected as false

  rospy.loginfo('object_detection.py initialization complete')

  # main loop
  while not rospy.is_shutdown():
    # retrieve frame
    ret, frame = cap.read()

    # rotate because of inverted camera
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    if not ret:
      # if camera closed, try reopening camera
      rospy.logerr('Camera unexpectedly closed, trying to reopen camera...')
      cap = cv2.VideoCapture(0) 

    if not RECORD_POST_EFFECT:
      # write frame video before drawings
      out.write(frame)
    
    # hitung range area untuk filter kontur yang diinginkan
    MIN_AREA, MAX_AREA = calc_obj_min_max_area(alt - DROP_ZONE_ELEVATION, DROP_ZONE_REAL_WIDTH)
    
    if (DROP_ZONE_START_WP <= wp_reached < DROP_ZONE_END_WP) or  (DROP_ZONE_START_WP2 <= wp_reached < DROP_ZONE_END_WP2):
        rospy.loginfo_throttle(60, "Object detection active")

        # detect target on image
        target = get_contour(frame, LOWER_H, UPPER_H, LOWER_S, UPPER_S, LOWER_V, UPPER_V, VERTICES, MIN_AREA, MAX_AREA)

        # check if there is target is detected
        if target[0]>=0:
          cx, cy, approx = target

          center_of_target = [cx,cy]

          rospy.loginfo(f'Object detected with center on ({cx},{cy})')

          target_dist = calc_ground_dist(center_of_target, center_of_frame, alt - DROP_ZONE_ELEVATION, CAM_ANGLE - pitch)
          #target_dist2 = calc_ground_dist_2(DROP_ZONE_REAL_WIDTH, obj_width, alt - DROP_ZONE_ELEVATION)

          # publish detection data
          target_dist_msg = Float32()
          target_dist_msg.data = target_dist
          target_dist_pub.publish(target_dist_msg)

          target_x_msg = Float32()
          target_x_msg.data = cx / frame_width
          target_x_pub.publish(target_x_msg)
          is_target_detected_pub.publish(is_target_detected_true)

          # draw contours on frame
          cv2.drawContours(frame, approx, 0, (0, 255, 0), 2)
          cv2.circle(frame, (cx, cy), 3, (255, 255, 255), -1)

          # write target distance from plane
          cv2.putText(frame,
                      'Dist : %.2f' %target_dist + ' m',
                      (cx+10, cy+5),
                      cv2.FONT_HERSHEY_SIMPLEX,
                      0.5,
                      (255, 0, 0),
                      2)

          # write detection information
          cv2.putText(frame, 
                      f'DETECTED: TRUE',
                      (10, 20), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      0.5, 
                      (0, 255, 0), 
                      2)
                             
          if is_target_aligned:
            cv2.putText(frame,
                  'TARGET ALIGNED: TRUE',
                  (10, 200),
                  cv2.FONT_HERSHEY_SIMPLEX,
                  0.5,
                  (0, 255, 0),
                  2)
          else:
            cv2.putText(frame,
                  'TARGET ALIGNED: FALSE',
                  (10, 200),
                  cv2.FONT_HERSHEY_SIMPLEX,
                  0.5,
                  (0, 0, 255),
                  2)
          
          cv2.putText(frame,
                'DROP TRAVEL DIST: %.2f' %drop_travel_dist,
                (10, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

        else:
          # when no target detected, publish
          is_target_detected_pub.publish(is_target_detected_false)
          # write detection information
          cv2.putText(frame, 
                      f'DETECTED: FALSE',
                      (10, 20), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      0.5, 
                      (0, 0, 255), 
                      2) 

    # after dropping, object detection not active
    else:
      rospy.loginfo_throttle(60, "Object detection not needed and inactive")  # logged every 1 minute
      is_target_detected_pub.publish(is_target_detected_false)                # publish that no object is detected
      cv2.putText(frame, 
                  f'DETECTED: FALSE',
                  (10, 20), 
                  cv2.FONT_HERSHEY_SIMPLEX, 
                  0.5, 
                  (0, 0, 255), 
                  2) 


    # on every frame
    # write other additional informations
    if fcu_state.connected:
      cur_mode = fcu_state.mode
      fcu_status_color = (0, 255, 0)
    
    else:
      cur_mode = 'DISCONNECTED'
      fcu_status_color = (0, 0, 255)
    
    if is_payload_dropped:
      cv2.putText(frame, 
                f'PAYLOAD DROPPED: TRUE',
                (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 255, 0),
                2)
    
    else:
      cv2.putText(frame, 
                f'PAYLOAD DROPPED: FALSE',
                (10, 40), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                (0, 0, 255),
                2)

    cv2.putText(frame, 
                f'MODE: {cur_mode}',
                (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.5, 
                fcu_status_color,
                2) 

    cv2.putText(frame,
                'REACHED WP: %.2f' % wp_reached,
                (10, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                'ALTITUDE: %.2f' % alt,
                (10, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                'GROUNDSPEED: %.2f' % groundspeed,
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    cv2.putText(frame,
                'RUDDER OUT PWM: %.2f' % rudder_out_pwm,
                (10, 140),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)
    
    cv2.putText(frame,
                'PITCH: %.2f' %pitch,
                (10, 160),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2)

    if RECORD_POST_EFFECT:
      # write frame video after drawings
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