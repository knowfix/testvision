#!/usr/bin/python3
import time
import rospy
from std_msgs.msg import Float64, Float32, Int32, Bool, Header
from mavros_msgs.msg import State, WaypointReached, OverrideRCIn, VFR_HUD
from geometry_msgs.msg import Quaternion, Vector3 
from calculation.free_fall import calc_horizontal_travel_dist
from target_detection import APPROACH_DROPPING_WP_INDEX, DROP_ZONE_ELEVATION, DROP_ZONE_END_WP
from simple_pid import PID

LOOP_RATE = 30

RUDDER_CHANNEL_INDEX = 3

ACCEPTABLE_DEVIATION = 0.15
DETECTING_TIME_LIMIT = 7

DROP_MECHANISM_CHANNEL_INDEX = 5
DROP_MECHANISM_PWM = 1900

MECHANISM_DELAY_ASSUMPTION = 1    # delay mekanisme dropping

if __name__ == '__main__':
    # initialize ros node
  rospy.loginfo('Initializing control.py...')
  rospy.loginfo('Initializing control node...')
  rospy.init_node('control', anonymous=True)

  # initialize target loop rate
  rate = rospy.Rate(LOOP_RATE)


  # initialize publishers
  rospy.loginfo('Initializing publishers...')
  
  is_aligned = False
  is_aligned_pub = rospy.Publisher('/fiachra/control/is_aligned', Bool, queue_size=10)
  # predefined topic message
  is_aligned_true = Bool()
  is_aligned_true.data = True
  is_aligned_false = Bool()
  is_aligned_false.data = False
  is_aligned_pub.publish(is_aligned_false)

  is_payload_dropped = False
  is_payload_dropped_pub = rospy.Publisher('/fiachra/control/is_payload_dropped', Bool, queue_size=10)
  # predefined topic message
  is_payload_dropped_false = Bool()
  is_payload_dropped_false.data = False
  is_payload_dropped_pub.publish(is_payload_dropped_false)

  override_rc_in_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

  drop_travel_dist_pub = rospy.Publisher('/fiachra/control/drop_travel_dist', Float32, queue_size=10)
  drop_travel_dist_msg = Float32()
  drop_travel_dist_msg.data = 999
  drop_travel_dist_pub.publish(drop_travel_dist_msg)


  # initialize subscribers
  rospy.loginfo('Initializing subscribers...')

  fcu_state = State()
  def fcu_state_cb(state):
    global fcu_state
    fcu_state = state
  rospy.Subscriber('/mavros/state', State, fcu_state_cb)

  wp_reached = 0
  def wp_reached_cb(msg):
    global wp_reached
    wp_reached = msg.wp_seq
  rospy.Subscriber('/mavros/mission/reached', WaypointReached, wp_reached_cb)

  groundspeed = 0
  airspeed = 0
  def vfr_cb(msg):
    global groundspeed, airspeed
    groundspeed = msg.groundspeed
    airspeed = msg.airspeed
  rospy.Subscriber('/mavros/vfr_hud', VFR_HUD, vfr_cb)

  target_dist = 999
  def target_dist_cb(msg):
    global target_dist
    target_dist = msg.data
  rospy.Subscriber('/fiachra/vision/target_dist', Float32, target_dist_cb)

  alt = 0
  def alt_cb(msg):
    global alt
    alt = msg.data
  rospy.Subscriber('/mavros/global_position/rel_alt', Float64, alt_cb)

  is_target_detected = False
  def is_target_detected_cb(msg):
    global is_target_detected
    is_target_detected = msg.data
  rospy.Subscriber('/fiachra/vision/is_target_detected', Bool, is_target_detected_cb)

  target_x = 0.5  # default center
  def target_x_cb(msg):
    global target_x
    target_x = msg.data
  rospy.Subscriber('/fiachra/vision/target_x', Float32, target_x_cb)


  # initialize helper functions
  rospy.loginfo('Initializing helper functions...')
  
  def set_override_pwm(rudder_pwm):
    global is_payload_dropped

    override_rc_in_msg = OverrideRCIn()
    if rudder_pwm > 0:
      override_rc_in_msg.channels[RUDDER_CHANNEL_INDEX] = int(rudder_pwm)
      rospy.loginfo(f'Rudder pwm is set to {rudder_pwm}')
    
    if is_payload_dropped:
      override_rc_in_msg.channels[DROP_MECHANISM_CHANNEL_INDEX] = int(DROP_MECHANISM_PWM)
    
    override_rc_in_pub.publish(override_rc_in_msg)
  
  def trigger_drop():
    global is_payload_dropped
    # update dropping mechanism
    is_payload_dropped = True

    # publish update
    is_payload_dropped_true = Bool()
    is_payload_dropped_true.data = True
    is_payload_dropped_pub.publish(is_payload_dropped_true)
    rospy.loginfo(f'======Payload dropped======')

  def failsafe_drop():
    global is_payload_dropped
    # update dropping mechanism
    is_payload_dropped = True
    
    # publish update
    is_payload_dropped_true = Bool()
    is_payload_dropped_true.data = True
    is_payload_dropped_pub.publish(is_payload_dropped_true)
    rospy.loginfo(f'======Payload dropped======')

    # set override pwm with is_payload_dropped variable value is true so payload dropped
    # zero the override rudder pwm so that it is ignored
    set_override_pwm(0)

# start main loop
  rospy.loginfo('control.py initialization complete')

  yaw_pid = PID(Kp=1500, Ki=0, Kd=400, sample_time=1/LOOP_RATE, output_limits=(-400,400))
  
  while not rospy.is_shutdown():
    if fcu_state.connected:
      # check if it is passing through the drop zone
      # store the time the current wp start
      if wp_reached == APPROACH_DROPPING_WP_INDEX-1:
        rospy.loginfo('===============ENTER DROP ZONE===============')
        wp_start_time = rospy.Time.now()

      # check whether the wp reached is in accordance with DROPPING WP
      if APPROACH_DROPPING_WP_INDEX <= wp_reached < DROP_ZONE_END_WP and fcu_state.mode == "AUTO":
        # store the current time 
        # and calculate the time spent while detecting and centering
        current_time = rospy.Time.now()
        centering_time_spent = current_time - wp_start_time

        # calculate the horizontal travel distance of payload
        drop_travel_dist = calc_horizontal_travel_dist(groundspeed, alt - DROP_ZONE_ELEVATION)

        # pengurangan estimasi travel payload karena headwind
        def TRAVEL_REDUCTED(gs, asp):
          wind = gs - asp
          return wind
        

        # calculate the horizontal travel distance of payload with external factors
        drop_travel_dist = drop_travel_dist + (groundspeed * MECHANISM_DELAY_ASSUMPTION) - TRAVEL_REDUCTED(groundspeed, airspeed)

        # publish data
        drop_travel_dist_msg = Float32()
        drop_travel_dist_msg.data = drop_travel_dist
        drop_travel_dist_pub.publish(drop_travel_dist_msg)

        # if object is detected
        # horizontally center the yaw to the drop zone as the plane is approaching
        if is_target_detected:
          deviation = target_x - 0.5
          rudder_pwm = 1500 - yaw_pid(deviation)
          rospy.loginfo(f'Deviation = {deviation}')
          set_override_pwm(rudder_pwm)
          # publish whether the object is aligned with the plane

          if deviation < ACCEPTABLE_DEVIATION and deviation > -ACCEPTABLE_DEVIATION:
            is_aligned = True
            is_aligned_pub.publish(is_aligned_true)
            rospy.loginfo('Target aligned, payload dropping ready')
            
          else:
            is_aligned = False
            is_aligned_pub.publish(is_aligned_false)
        
        # if no target is detected
        # publish the alignment to false
        else:
          is_aligned = False
          is_aligned_pub.publish(is_aligned_false)

        # check if it has been too long since the detecting and centering start
        # if so, run fail safe
        if centering_time_spent.secs > DETECTING_TIME_LIMIT:
          # force drop payload
          rospy.logwarn('FAIL SAFE: Force dropping payload')
          failsafe_drop()

        # check if all the conditions for dropping is satisfied
        elif is_target_detected and target_dist <= drop_travel_dist and is_aligned:
          trigger_drop()

        else:
          # passing so that it is ignored
          pass

      else:
        # passing so that it is ignored
        pass

    else:
      rospy.logerr('Disconnected from FCU')

    try:
      rate.sleep()
    except:
      rospy.logwarn('rate.sleep() interrupted by shutdown.')

  # program finished
  rospy.loginfo('control.py is shut down')