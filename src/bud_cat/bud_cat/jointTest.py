import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import rclpy
from rclpy.node import Node

import time
import adafruit_pca9685
from adafruit_servokit import ServoKit
import serial
import board
import busio

kit = ServoKit(channels=16, address=0x40)
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

#kit.set_pwm_freq(50)
#pca.setPWMFreq(50)
pca.frequency = 100
print("test all servos")
maxr=135
minl=30
maxthr=125
minthr= 65
thrinit = 90
strinit = 10


KneeLF = kit.servo[12] 
ArmLF = kit.servo[8]
ShoulderLF = kit.servo[4]

class JointTestNode(Node):

    def __init__(self):
        super().__init__('joint_test_node')
        self.sub = self.create_subscription(Twist, 'joint_states', self.listener_callback,10)

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    jointTest = JointTestNode()

    rclpy.spin(jointTest)

    jointTest.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
