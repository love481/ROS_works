import rclpy
from rclpy.node import Node
import serial,math
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np

class To_Motor:
    def __init__(self):
        self.Comport = serial.Serial('/dev/ttyUSB_motor')  # open COM
        self.Comport.baudrate = 9600  # set Baud rate to 9600
        self.Comport.bytesize = 8  # Number of data bits = 8
        self.Comport.parity = 'N'  # No parity
        self.Comport.stopbits = 1  # Number of Stop bits = 1
        self.Comport.timeout = 5  # self.Comport.open()
        self.r = 0.1  # wheel radius
        self.l = 0.33  # wheel base
        self.max_speed_percentage=80

    def send_cmd(self, v, w):
        v = -v
        w = w
        w_r = (2 * v + w * self.l) / (2 * self.r)
        w_l = (2 * v - w * self.l) / (2 * self.r)

        rpm_1 = (w_l * 60) / (2 * math.pi)
        rpm_2 = -(w_r * 60) / (2 * math.pi)

        dec_val = round(65535 - (self.max_speed_percentage / 100) * 65535)
        hex_val = "0x{:04x}".format(dec_val)
        m10 = hex_val[2:4]
        m9 = hex_val[4:6]

        ppm_val_1 = round((410 / 6) * rpm_1)
        ppm_val_2 = round((410 / 6) * rpm_2)
        if ppm_val_1 < 0:
            s = bin(ppm_val_1 & int("1" * 16, 2))[2:]
            bin_v_1 = ("{0:0>%s}" % 16).format(s)
        else:
            bin_v_1 = '{0:016b}'.format(ppm_val_1)

        if ppm_val_2 < 0:
            s = bin(ppm_val_2 & int("1" * 16, 2))[2:]
            bin_v_2 = ("{0:0>%s}" % 16).format(s)
        else:
            bin_v_2 = '{0:016b}'.format(ppm_val_2)

        b3_1 = (bin_v_1[0:4])
        b4_1 = (bin_v_1[4:8])
        b1_1 = (bin_v_1[8:12])
        b2_1 = (bin_v_1[12:16])

        b3_2 = (bin_v_2[0:4])
        b4_2 = (bin_v_2[4:8])
        b1_2 = (bin_v_2[8:12])
        b2_2 = (bin_v_2[12:16])

        def bin2hex(n):
            decimal_representation = int(n, 2)
            hexadecimal_string = "0x{:02x}".format(decimal_representation)
            return hexadecimal_string

        h1_1 = bin2hex(b1_1)[3:4]
        h2_1 = bin2hex(b2_1)[3:4]
        h3_1 = bin2hex(b3_1)[3:4]
        h4_1 = bin2hex(b4_1)[3:4]

        h1_2 = bin2hex(b1_2)[3:4]
        h2_2 = bin2hex(b2_2)[3:4]
        h3_2 = bin2hex(b3_2)[3:4]
        h4_2 = bin2hex(b4_2)[3:4]

        m13 = h1_2 + h2_2
        m14 = h3_2 + h4_2
        m11 = h1_1 + h2_1
        m12 = h3_1 + h4_1

        m1 = 'AA'
        m2 = 'A1'
        m3 = '0E'
        m4 = '00'
        m5 = '00'
        m6 = '80'
        m7 = 'C8'
        m8 = '00'

        summed = hex(
            int(m1, 16) + int(m2, 16) + int(m3, 16) + int(m4, 16) + int(m5, 16) + int(m6, 16) + int(m7, 16) + int(m8,
                                                                                                                  16) + int(
                m9, 16) + int(m10, 16) + int(m11, 16) + int(m12, 16) + int(m13, 16) + int(m14, 16))
        summed = summed[2:]
        sum_dec = int(summed, 16)
        sum_bin = '{0:032b}'.format(sum_dec)
        m15 = bin2hex(sum_bin[24:])[2:]
        lis = m1 + m2 + m3 + m4 + m5 + m6 + m7 + m8 + m9 + m10 + m11 + m12 + m13 + m14 + m15
        msg = bytearray.fromhex(lis)
        self.Comport.write(msg)

    def get_odom(self, rob_states, enc_states, run, dt):
        x = rob_states[0]
        y = rob_states[1]
        th = rob_states[2]
        encl_last = enc_states[0]
        encr_last = enc_states[1]
        enc1_first = enc_states[2]
        enc2_first = enc_states[3]
        datain = self.Comport.read(15)
        if datain:
            pass
        else:
            return None
        if datain[9] == 255:
            if run == 0:
                e1b1_first = "0x{:02x}".format(datain[9])[2:4]
                e1b2_first = "0x{:02x}".format(datain[8])[2:4]
                e1b3_first = "0x{:02x}".format(datain[7])[2:4]
                e1b4_first = "0x{:02x}".format(datain[6])[2:4]
                enc1_hex_first = e1b1_first + e1b2_first + e1b3_first + e1b4_first
                enc1_first = int(enc1_hex_first, 16) - 4294967296
            e1b1 = "0x{:02x}".format(datain[9])[2:4]
            e1b2 = "0x{:02x}".format(datain[8])[2:4]
            e1b3 = "0x{:02x}".format(datain[7])[2:4]
            e1b4 = "0x{:02x}".format(datain[6])[2:4]
            enc1_hex = e1b1 + e1b2 + e1b3 + e1b4
            enc1_cur = int(enc1_hex, 16) - 4294967296
        else:
            if run == 0:
                e1b1_first = "0x{:02x}".format(datain[9])[2:4]
                e1b2_first = "0x{:02x}".format(datain[8])[2:4]
                e1b3_first = "0x{:02x}".format(datain[7])[2:4]
                e1b4_first = "0x{:02x}".format(datain[6])[2:4]
                enc1_hex_first = e1b1_first + e1b2_first + e1b3_first + e1b4_first
                enc1_first = int(enc1_hex_first, 16)
            e1b1 = "0x{:02x}".format(datain[9])[2:4]
            e1b2 = "0x{:02x}".format(datain[8])[2:4]
            e1b3 = "0x{:02x}".format(datain[7])[2:4]
            e1b4 = "0x{:02x}".format(datain[6])[2:4]
            enc1_hex = e1b1 + e1b2 + e1b3 + e1b4
            enc1_cur = int(enc1_hex, 16)
        if datain[13] == 255:
            if run == 0:
                e2b1_first = "0x{:02x}".format(datain[13])[2:4]
                e2b2_first = "0x{:02x}".format(datain[12])[2:4]
                e2b3_first = "0x{:02x}".format(datain[11])[2:4]
                e2b4_first = "0x{:02x}".format(datain[10])[2:4]
                enc2_hex_first = e2b1_first + e2b2_first + e2b3_first + e2b4_first
                enc2_first = int(enc2_hex_first, 16) - 4294967296
            e2b1 = "0x{:02x}".format(datain[13])[2:4]
            e2b2 = "0x{:02x}".format(datain[12])[2:4]
            e2b3 = "0x{:02x}".format(datain[11])[2:4]
            e2b4 = "0x{:02x}".format(datain[10])[2:4]
            enc2_hex = e2b1 + e2b2 + e2b3 + e2b4
            enc2_cur = int(enc2_hex, 16) - 4294967296
        else:
            if run == 0:
                e2b1_first = "0x{:02x}".format(datain[13])[2:4]
                e2b2_first = "0x{:02x}".format(datain[12])[2:4]
                e2b3_first = "0x{:02x}".format(datain[11])[2:4]
                e2b4_first = "0x{:02x}".format(datain[10])[2:4]
                enc2_hex_first = e2b1_first + e2b2_first + e2b3_first + e2b4_first
                enc2_first = int(enc2_hex_first, 16)
            e2b1 = "0x{:02x}".format(datain[13])[2:4]
            e2b2 = "0x{:02x}".format(datain[12])[2:4]
            e2b3 = "0x{:02x}".format(datain[11])[2:4]
            e2b4 = "0x{:02x}".format(datain[10])[2:4]
            enc2_hex = e2b1 + e2b2 + e2b3 + e2b4
            enc2_cur = int(enc2_hex, 16)

        enc1 = enc1_cur - enc1_first
        enc2 = enc2_cur - enc2_first
        enc_l = enc1 - encl_last
        enc_r = enc2 - encr_last

        deg1 = (360 / 4096) * enc_l
        deg2 = -(360 / 4096) * enc_r
        dist_l = ((2 * math.pi * self.r) / 360) * deg1
        dist_r = ((2 * math.pi * self.r) / 360) * deg2
        dist_mean = (dist_l + dist_r) / 2
        x = x + dist_mean * math.cos(th)
        y = y + dist_mean * math.sin(th)
        th = th + (dist_r - dist_l) / self.l

        # dt = (current_time - last_time).to_sec()
        v_x = (dist_mean * math.cos(th)) / dt
        v_y = 0.0
        v_th = ((dist_r - dist_l) / self.l) / dt

        if th > math.pi:
            th = (th - math.pi) - math.pi
        if th < -math.pi:
            th = (th + math.pi) + math.pi

        states_rob = [x, y, th, v_x, v_y, v_th]
        states_enc = [enc1, enc2, enc1_first, enc2_first]
        return states_rob, states_enc


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class odom_publisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry, 'odom', 50)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period , self.timer_callback)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.odom= Odometry()
        self.t = TransformStamped()
        self.time_initial = self.get_clock().now()
        self.rob_states=[0, 0, 0, 0, 0, 0]
    ## get the robot_states from the current states
    def set_robo_states(self,rob_states):
        self.rob_states=rob_states
    def timer_callback(self):
        x = -self.rob_states[0]
        y = -self.rob_states[1]
        th = self.rob_states[2]
        v_x = self.rob_states[3]
        v_y = self.rob_states[4]
        v_th = self.rob_states[5]
        q = quaternion_from_euler(0, 0, th)
        self.t = TransformStamped()
        self.t.header.stamp=self.get_clock().now().to_msg()
        self.t.header.frame_id="odom"
        self.t.child_frame_id="base_link"
        self.t.transform.translation.x=x
        self.t.transform.translation.y=y
        self.t.transform.translation.z=0.0
        self.t.transform.rotation.x = q[0]
        self.t.transform.rotation.y = q[1]
        self.t.transform.rotation.z = q[2]
        self.t.transform.rotation.w = q[3]
        self.odom_broadcaster.sendTransform(self.t)

        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.header.frame_id = "odom"
        self.odom.pose.pose.position.x=x
        self.odom.pose.pose.position.y=y
        self.odom.pose.pose.position.z=0.0
        self.odom.pose.pose.orientation.x=q[0]
        self.odom.pose.pose.orientation.y=q[1]
        self.odom.pose.pose.orientation.z=q[2]
        self.odom.pose.pose.orientation.w=q[3]



        self.odom.child_frame_id = "base_link"

        self.odom.twist.twist.linear.x=v_x
        self.odom.twist.twist.linear.y=v_y
        self.odom.twist.twist.linear.z=0.0
        self.odom.twist.twist.angular.x=0.0
        self.odom.twist.twist.angular.y=0.0
        self.odom.twist.twist.angular.z=v_th
        self.publisher_.publish(self.odom)

class cmd_vel_subscriber(Node):
    def __init__(self,odom_handle):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.motor=To_Motor()
        self.subscription  # prevent unused variable warning
        self.count=0
        self.rob_states=[0, 0, 0, 0, 0, 0]
        self.enc_states=[0, 0, 0, 0]
        self.last_time=self.get_clock().now()
        self.odom_handle= odom_handle

    def cmd_vel_callback(self, data):
        v = data.linear.x
        w = data.angular.z
        self.motor.send_cmd(v*0.8, w*0.8)
        dt = (self.get_clock().now()- self.last_time).nanoseconds/1000000000.0
        self.rob_states, self.enc_states = self.motor.get_odom(self.rob_states,self.enc_states, self.count, dt)
        self.odom_handle.set_robo_states(self.rob_states)
        self.count += 1
        self.last_time=self.get_clock().now()



def main(args=None):
    rclpy.init(args=args)
    odom_handle= odom_publisher()
    cmd_vel_handle = cmd_vel_subscriber(odom_handle)
    while rclpy.ok():
        rclpy.spin_once(cmd_vel_handle)
        rclpy.spin_once(odom_handle)
    odom_handle.destroy_node()
    cmd_vel_handle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
