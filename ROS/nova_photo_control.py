#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Twist
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import SetMode, CommandBool
import math
import cv2
from cv_bridge import CvBridge
import os

class AUVController:
    def __init__(self):
        rospy.init_node('imu_controlled_auv', anonymous=True)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
        self.pwm_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        self.current_orientation = None
        self.current_state = None
        self.armed = False
        self.stabilize_mode = False
        self.rate = rospy.Rate(10)  # 10 Hz
        self.bridge = CvBridge()
        self.image_dir = "/home/raclabnova/nova/images"
        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)
        self.image_count = 0

    def imu_callback(self, data):
        self.current_orientation = data.orientation

    def state_callback(self, state):
        self.current_state = state

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_path = os.path.join(self.image_dir, "image_{}.jpg".format(self.image_count))
            cv2.imwrite(image_path, cv_image)
            rospy.loginfo("Saved image: {}".format(image_path))
            self.image_count += 1
        except Exception as e:
            rospy.logerr("Failed to save image: %s" % e)

    def get_yaw(self):
        if self.current_orientation is None:
            return None
        orientation_q = self.current_orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def arm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            response = arm_service(True)
            if response.success:
                rospy.loginfo("Vehicle armed")
                self.armed = True
            else:
                rospy.logerr("Failed to arm vehicle")
        except rospy.ServiceException as e:
            rospy.logerr("Arming service call failed: %s" % e)

    def set_mode(self, mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = mode_service(custom_mode=mode)
            if response.mode_sent:
                rospy.loginfo("Mode set to %s" % mode)
                if mode == "STABILIZE":
                    self.stabilize_mode = True
            else:
                rospy.logerr("Failed to set mode to %s" % mode)
        except rospy.ServiceException as e:
            rospy.logerr("Set mode service call failed: %s" % e)

    def move_distance(self, distance):
        velocity_msg = Twist()
        velocity_msg.linear.x = 1.0
        self.vel_pub.publish(velocity_msg)
        rospy.loginfo("Moving forward...")

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < distance:
            self.rate.sleep()

        velocity_msg.linear.x = 0.0
        self.vel_pub.publish(velocity_msg)
        rospy.loginfo("Stopped.")

    def turn_angle(self, angle):
        initial_yaw = self.get_yaw()
        if initial_yaw is None:
            rospy.logerr("IMU data not available")
            return

        rospy.loginfo("Starting turn: Initial yaw angle: %f" % initial_yaw)
        target_yaw = initial_yaw + math.radians(angle)
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        elif target_yaw < -math.pi:
            target_yaw += 2 * math.pi

        velocity_msg = Twist()
        velocity_msg.angular.z = 0.5  # Turn rate

        while not rospy.is_shutdown():
            current_yaw = self.get_yaw()
            if current_yaw is None:
                continue

            rospy.loginfo("Current yaw angle: %f, Target yaw angle: %f" % (current_yaw, target_yaw))
            if abs(current_yaw - target_yaw) < 0.01:
                rospy.loginfo("Reached target yaw angle: %f" % current_yaw)
                break

            self.vel_pub.publish(velocity_msg)
            self.rate.sleep()

        velocity_msg.angular.z = 0.0
        self.vel_pub.publish(velocity_msg)
        rospy.loginfo("Turn stopped.")

    def control_motors(self, motor_5_pwm, motor_6_pwm, duration):
        rc_msg = OverrideRCIn()
        rc_msg.channels = [1500] * 18  # Center all channels
        rc_msg.channels[4] = motor_5_pwm  # Motor 5 PWM value
        rc_msg.channels[5] = motor_6_pwm  # Motor 6 PWM value
        self.pwm_pub.publish(rc_msg)
        rospy.loginfo("Controlling motors: 5=%d, 6=%d" % (motor_5_pwm, motor_6_pwm))

        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.rate.sleep()

        rc_msg.channels[4] = 1500  # Stop motor 5
        rc_msg.channels[5] = 1500  # Stop motor 6
        self.pwm_pub.publish(rc_msg)
        rospy.loginfo("Motors stopped.")

    def run(self):
        rospy.loginfo("Control initiated...")
        self.arm()  # Arm the vehicle
        rospy.sleep(2)
        self.set_mode("STABILIZE")  # Set mode to STABILIZE
        rospy.sleep(2)
        
        self.control_motors(1600, 1600, 5)  # set motors 5 and 6 to 1600 PWM for 5 seconds
        rospy.sleep(5)
        self.move_distance(10)  # Move forward 
        rospy.sleep(5)  # Wait
        self.turn_angle(-90)  # Turn 90 degrees
        rospy.sleep(5)  # Wait
        self.move_distance(10)  # Move forward 
        rospy.sleep(5)  # Wait
        self.turn_angle(90)  # Turn 90 degrees to the left (back to original position)
        rospy.sleep(5)  # Wait for 5 seconds
        self.move_distance(10)  # Move forward 
        rospy.sleep(5)  # Wait
        
        self.set_mode("MANUAL")  # Set mode to MANUAL
        rospy.sleep(2)

        rospy.loginfo("Control completed.")


if __name__ == '__main__':
    try:
        controller = AUVController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
