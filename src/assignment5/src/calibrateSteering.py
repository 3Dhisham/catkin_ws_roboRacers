import rospy
from autominy_msgs.msg import Speed, SteeringAngle, SteeringFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose


class CalibrateSteering:

    # Init Coordinate points
    x_point = 0.0
    y_point = 0.0
    z_point = 0.0

    # Initiate node and subscribers
    def __init__(self):
        rospy.init_node("calibrate_steering")
        # This subscribes to the ceiling camera topic which prints out the points based on car location
        self.localization_subscriber = rospy.Subscriber("/communication/gps/5", Odometry, self.read_points, queue_size=10)
        # Steering angel topic
        # self.angle_subscriber = rospy.Subscriber("/sensors/arduino/steering_angle", SteeringFeedback, self.steering_angle,queue_size=10)
        self.speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)
        self.steering_publisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand,
                                              queue_size=10)
        # The spin() function runs the ros node in loops 
        rospy.spin()
        self.rate = rospy.Rate(10)  # 10hz

    # drives the car in a circle to record points
    def drive(self, steerVal, speedVal):
        steer_msg = NormalizedSteeringCommand()
        steer_msg.value = steerVal
        self.steering_publisher.publish(steer_msg)

        speed_msg = SpeedCommand()
        speed_msg.value = speedVal
        self.speed_publisher.publish(speed_msg)
        self.rate.sleep()


    # Extracts the points from the localization topic and prints them out
    def read_points(self, odometry):
        global x_point
        global y_point
        global z_point

        x_point = odometry.pose.pose.position.x
        y_point = odometry.pose.pose.position.y
        z_point = odometry.pose.pose.position.z

        print("=======================================================")
        print("x: ", x_point)
        print("y: ", y_point)
        print("z: ", z_point)
        print("=======================================================")


# In here, we need to let the car drive in a circle, and while it is driving and changing position, 
# we need to record three different positions with (x,y,z). 

# With these three points, we can calculate the radius and the steering angle.
# Steering angle can be calculated with 
# 	angle w = math.atan(radius/0.27)   //comment: 27 cm distanz zw hinter Achse und vorder Achse


if __name__ == '__main__':
    try:
        steering = CalibrateSteering()
        steering.drive(0.8, 0.1)
        #rospy.spin()
        #CalibrateSteering()
    except rospy.ROSInterruptException:
        pass
