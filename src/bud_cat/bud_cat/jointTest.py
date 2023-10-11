import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import time
import adafruit_pca9685
from adafruit_servokit import ServoKit
import serial
import board
import busio

kit = ServoKit(channels=16, address=0x40)
i2c = busio.I2C(board.SCL, board.SDA)
pca = adafruit_pca9685.PCA9685(i2c)

kit.set_pwm_freq(50)
pca.setPWMFreq(50)
pca.frequency = 100
print("test all servos")



maxr=135
minl=30
maxthr=125
minthr= 65
thrinit = 90
strinit = 10

angle1 = 0
angle2 = 0

KneeLF = kit.servo[12] 
ArmLF = kit.servo[8]
ShoulderLF = kit.servo[4]


def mapRange(value, leftMin=-math.pi, leftMax=0, rightMin=0, rightMax=math.pi):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

class JointTestNode(Node):

    def __init__(self):
        super().__init__('joint_test_node')
        self.sub = self.create_subscription(JointState, 'joint_states', self.listener_callback,10)

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.position[0])
        angle1 = msg.position[0]
        angle2 = msg.position[1]
        # self.get_logger().info(f'\n 1 :{angle1} \n 2 :{angle2}')
        self.get_logger().info(f'\n 0 :{msg.position[0]} \n 1 :{msg.position[1]} \n 2 :{msg.position[2]} \n 3 :{msg.position[3]} \n 4 :{msg.position[4]} \n 5 :{msg.position[5]} \n 6 :{msg.position[6]}')
        KneeLF.angle = mapRange(angle1)
        ArmLF.angle = mapRange(angle2)


def main(args=None):
    rclpy.init(args=args)

    jointTest = JointTestNode()

    rclpy.spin(jointTest)

    jointTest.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
