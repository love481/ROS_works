import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped,TransformStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import math



class Camerahandle(Node):

    def __init__(self):
        super().__init__('camera_handle')
        self.camera_data=CameraInfo()
        self.camera_depth=np.zeros((240,360), dtype=np.float32)
        self.cv_bridge = CvBridge()
        self.point_transformed=PointStamped()
        self.trans_c_b =TransformStamped()    ##base_link to camera_depth_frame
        self.trans_o_m =TransformStamped()    ##map to odom frame
        self.tf_buffer = [Buffer(),Buffer()]
        self.tf_listener = [TransformListener(self.tf_buffer[0], self),TransformListener(self.tf_buffer[1], self)]  ## for for each above transform  stamp
        self.robot_odom=Odometry()
        self.subscription_robot_odom = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.subscription_robot_odom
        self.subscription_depthpoint = self.create_subscription(
            Image,
            '/intel_realsense_r200_depth/depth/image_raw',
            self.cameradepthpoint_callback,
            10) # prevent unused variable warning
        self.subscription_depthpoint 
        self.subscription_camerainfo = self.create_subscription(
            CameraInfo,
            '/intel_realsense_r200_depth/depth/camera_info',
            self.camerainfo_callback,
            10)
        self.subscription_camerainfo
        self.publisher_ = self.create_publisher(PointStamped, 'd_point', 10)

    def odom_callback(self, msg):
        self.robot_odom=msg

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
    
    def quaternion_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

    def get_transform_for_camera_wrt_baselink(self):
        try:
            self.trans_c_b = self.tf_buffer[0].lookup_transform(
                "base_link",   ## target_frame
                "camera_depth_frame",  ##src frame
                rclpy.time.Time())
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform base_link to camera_depth_frame: {ex}')
            return
    def get_transform_for_odom_wrt_map(self):
        try:
            self.trans_o_m = self.tf_buffer[1].lookup_transform(
                "map",   ## target_frame
                "odom",  ##src frame
                rclpy.time.Time())
        except TransformException as ex:
            # self.get_logger().info(
            #     f'Could not transform map to odom: {ex}')
            return
        
    def camerainfo_callback(self, msg):
        self.get_transform_for_camera_wrt_baselink()
        self.get_transform_for_odom_wrt_map()
        self.camera_data=msg
        CX_DEPTH=msg.k[2]
        CY_DEPTH=msg.k[5]
        FX_DEPTH=msg.k[0]
        FY_DEPTH=msg.k[4]
        for i in range(msg.height):
            for j in range(msg.width):
                z = self.camera_depth[i,j]
                x = (j - CX_DEPTH) * z / FX_DEPTH
                y = (i - CY_DEPTH) * z / FY_DEPTH
                if i==120 and j==160:
                    self.point_transformed.header.frame_id="map"

                    ## from base_link to camera_depth_frame
                    Q=[self.trans_c_b.transform.rotation.x,
                    self.trans_c_b.transform.rotation.y,
                    self.trans_c_b.transform.rotation.z,
                    self.trans_c_b.transform.rotation.w]
                    R=self.quaternion_rotation_matrix(Q)
                    T=np.array([self.trans_c_b.transform.translation.x,self.trans_c_b.transform.translation.y,self.trans_c_b.transform.translation.z])
                    new_point_base_link=R@np.array([x,y,float(z)])+T

                    ## from odom to base_link
                    Q=[self.robot_odom.pose.pose.orientation.x,
                    self.robot_odom.pose.pose.orientation.y,
                    self.robot_odom.pose.pose.orientation.z,
                    self.robot_odom.pose.pose.orientation.w]
                    _,_,yaw=self.euler_from_quaternion(Q[0],Q[1],Q[2],Q[3])
                    T=np.array([self.robot_odom.pose.pose.position.x,self.robot_odom.pose.pose.position.y])
                    R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                    new_point_odom=R@new_point_base_link[:2]+T

                    ## from map to odom
                    Q=[self.trans_o_m.transform.rotation.x,
                    self.trans_o_m.transform.rotation.y,
                    self.trans_o_m.transform.rotation.z,
                    self.trans_o_m.transform.rotation.w]
                    _,_,yaw=self.euler_from_quaternion(Q[0],Q[1],Q[2],Q[3])
                    T=np.array([self.trans_o_m.transform.translation.x,self.trans_o_m.transform.translation.y])
                    R=np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
                    new_point_map=R@new_point_odom[:2]+T
                    self.point_transformed.point.x=new_point_map[0]
                    self.point_transformed.point.y=new_point_map[1]
                    self.point_transformed.point.z=new_point_base_link[2]
                    self.publisher_.publish(self.point_transformed)

    def cameradepthpoint_callback(self, msg):
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        self.camera_depth= np.array(depth_image, dtype=np.float32)




def main(args=None):
    rclpy.init(args=args)
    camera_handle = Camerahandle()
    rclpy.spin(camera_handle)
    camera_handle .destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
