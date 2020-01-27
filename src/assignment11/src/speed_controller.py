import rospy
import math
import numpy as np
import tf.transformations

from autominy_msgs.msg import SpeedCommand


class SpeedController:

    def __init__(self, wanted_speed):
        rospy.init_node("speed_controller")
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.sub_speed = rospy.Subscriber("/sensors/speed", SpeedCommand, self.callback_speed, queue_size=1)

        self.current_speed = 0.0
        self.wanted_speed = wanted_speed
        self.integral_error = 0.0
        self.last_error = 0.0

        self.kp = 0.8
        self.ki = 0.09
        self.kd = 0.05
        self.min_i = -1.0
        self.max_i = 1.0

        self.rate = rospy.Rate(100)
        self.tmr = rospy.Timer(rospy.Duration.from_sec(0.01), self.callback_control)

        while not rospy.is_shutdown():
            self.rate.sleep()


    def callback_speed(self, msg):
        self.current_speed = msg.value

    def callback_control(self, tmr):

        if tmr.last_duration is None:
            dt = 0.01
        else:
            dt = (tmr.current_expected - tmr.last_expected).to_sec()


        if self.wanted_speed is None:
            print("Wanted speed is not set")
            print("Exited")
            return

        error = np.abs(self.wanted_speed - self.current_speed)
        print("Current Speed is: " + str(self.current_speed))

        self.integral_error += error * dt
        self.integral_error = max(self.min_i, self.integral_error)
        self.integral_error = min(self.max_i, self.integral_error)

        derivative_error = (error - self.last_error) / dt
        self.last_error = error

        controller_output = self.kp * error + self.kd * derivative_error

        speed_msg = SpeedCommand()
        speed_msg.value = controller_output
        speed_msg.header.frame_id = "base_link"

        self.pub_speed.publish(speed_msg)



if __name__ == "__main__":
    wanted_sp = input("Please type Wanted Speed between 0.0 and 1.0: ")
    SpeedController(wanted_sp)
