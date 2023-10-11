import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from math import pi
class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)

        loop_rate = self.create_rate(30)

        # robot state

        thigh_FR_angle = 0.0
        leg_FR_angle = 0.0
        thigh_BR_angle = 0.0
        leg_BR_angle = 0.0
        thigh_FL_angle = 0.0
        leg_FL_angle = 0.0
        thigh_BL_angle = 0.0
        leg_BL_angle = 0.0
        i1= 0.01
        i2 = 0.03
        

        # message declarations
        joint_state = JointState()

        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['thigh_FR_joint','leg_FR_joint','thigh_BR_joint','leg_BR_joint','thigh_FL_joint','leg_FL_joint','thigh_BL_joint','leg_BL_joint']
                joint_state.position = [thigh_FR_angle,leg_FR_angle,thigh_BR_angle,leg_BR_angle,thigh_FL_angle,leg_FL_angle,thigh_BL_angle,leg_BL_angle]
                self.joint_pub.publish(joint_state)
                
                if(thigh_FR_angle <= -1.57 or thigh_FR_angle >= 1.57):
                    i1 = -i1 
                thigh_FR_angle = thigh_FR_angle +i1  

                if(leg_FR_angle <= -pi or leg_FR_angle >= 0):
                    i2 = -i2 
                leg_FR_angle += i2

                # This will adjust as needed per iteration
                loop_rate.sleep()

        except KeyboardInterrupt:
            pass

def main():
    StatePublisher()

if __name__ == '__main__':
    main()