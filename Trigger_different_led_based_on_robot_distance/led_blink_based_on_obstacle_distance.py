import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped,Point,PoseStamped,PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from tf2_ros.buffer import Buffer
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import math
from math import cos, sin

#for led operations or we can visualize the point stamped on the rviz
import Jetson.GPIO as GPIO
import time 
#green(> 1.5m),blue(0.5 > and <1.5m), red(<0.5) or stuck
led_pins_obstacle = [18, 12, 13]
yellow following
led_follow_pin = 5
 
# SET UP THE GPIO CHANNEL
GPIO.SETMODE(GPIO.BOARD) 
GPIO.SETUP(LED_PINS_OBSTACLE, GPIO.OUT) 
GPIO.SETUP(LED_FOLLOW_PIN, GPIO.OUT)
        

class Subscriber_laser_pose(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10)
        self.subscription_laser
        self.subscription_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription_cmd_vel  # prevent unused variable warning
        self.laser=LaserScan()
        self.nearest_obstacle_point = PoseStamped()
        self.velocity_update = Twist()

        self.publisher_ns = self.create_publisher(PointStamped, '/closest_obstacle_point_ns', 50)
        self.publisher_s = self.create_publisher(PointStamped, '/closest_obstacle_point_s', 50)
        self.publisher_r = self.create_publisher(PointStamped, '/closest_obstacle_point_r', 50)
        self.publisher_g = self.create_publisher(PointStamped, '/closest_obstacle_point_g', 50)
        self.publisher_b = self.create_publisher(PointStamped, '/closest_obstacle_point_b', 50)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.take_action)



    def get_transform(self):
        try:
            self.trans = self.tf_buffer.lookup_transform(
                "base_link",   ## target_frame
                "base_scan",  ##src frame
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_scan to laser_link: {ex}')
            return


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    def take_action(self):
        laser=self.laser.ranges
        shortest_laser=10000
        point=Point()
        for i in range(len(laser)):
            if laser[i] <shortest_laser:
                angle=self.laser.angle_min + i*self.laser.angle_increment
                x=laser[i]*cos(angle)
                if x>-0.2:
                    shortest_laser=laser[i]
                    point.x=x
                    point.y= shortest_laser*sin(angle)
        point.z=0.0
        _, _, yaw = self.euler_from_quaternion(
        self.trans.transform.rotation.x,
        self.trans.transform.rotation.y,
        self.trans.transform.rotation.z,
        self.trans.transform.rotation.w)    
        self.nearest_obstacle_point.header=self.laser.header
        self.nearest_obstacle_point.pose.position.x=cos(yaw)*point.x - sin(yaw)*point.y + self.trans.transform.translation.x
        self.nearest_obstacle_point.pose.position.y=sin(yaw)*point.x + cos(yaw)*point.y + self.trans.transform.translation.y
        self.nearest_obstacle_point.pose.position.z=0.0
        point_transformed=PointStamped()
        point_transformed.header=  self.nearest_obstacle_point.header
        point_transformed.point= self.nearest_obstacle_point.pose.position
        d_r= math.sqrt((self.nearest_obstacle_point.pose.position.x)**2 + (self.nearest_obstacle_point.pose.position.y)**2)
        robot_is_stuck=False
        print(d_r)


        if (d_r >1.5):
            GPIO.output(led_pins_obstacle, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))
            self.publisher_g.publish(point_transformed)

        elif (d_r <= 1.5 and d_r >0.5):
            GPIO.output(led_pins_obstacle, (GPIO.LOW, GPIO.HIGH, GPIO.LOW))
            self.publisher_b.publish(point_transformed)

        elif (d_r <= 0.5):
            if (self.velocity_update.linear.x < 0.01 and self.velocity_update.angular.x < 0.01):
                robot_is_stuck=True
            else:
                robot_is_stuck=False
            if robot_is_stuck:
                GPIO.output(led_pins_obstacle, (GPIO.LOW, GPIO.LOW, GPIO.HIGH))
                self.publisher_s.publish(point_transformed)
            else:
                GPIO.output(led_pins_obstacle, (GPIO.LOW, GPIO.LOW, GPIO.HIGH))
                time.sleep(2) 
                GPIO.output(led_pins_obstacle, (GPIO.LOW, GPIO.LOW, GPIO.LOW))
                time.sleep(2)
                self.publisher_r.publish(point_transformed)
        if not robot_is_stuck:
            GPIO.output(led_follow_pin,GPIO.HIGH)
            time.sleep(2) 
            GPIO.output(led_follow_pin,GPIO.LOW)
            time.sleep(2)
            print("not_stucked")
            self.publisher_ns.publish(point_transformed)



       
        #print(point_transformed)

    def laser_callback(self, msg):
        self.laser=msg
        self.get_transform()
    def  cmd_vel_callback(self,msg):
        self.velocity_update=msg



def main(args=None):
    rclpy.init(args=args)
    laser_pose_subscriber = Subscriber_laser_pose()
    rclpy.spin(laser_pose_subscriber)
    laser_pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

