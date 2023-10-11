import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import math

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)

    def inverse_kinematics(self, x, y):
        # Your inverse kinematics calculation here...
        # Replace this with the actual inverse kinematics solver for your robot

        # Example: Simple 2-DOF robotic arm inverse kinematics
        L1 = 5.0
        L2 = 5.0
        distance = math.sqrt(x**2 + y**2)
        shoulder_angle = math.atan2(y, x)
        cos_theta2 = (distance**2 - L1**2 - L2**2) / (2 * L1 * L2)
        sin_theta2 = math.sqrt(1 - cos_theta2**2)
        elbow_angle = math.atan2(sin_theta2, cos_theta2)
        
        return shoulder_angle, elbow_angle

    def publish_joint_angles(self, x, y):
        shoulder_angle, elbow_angle = self.inverse_kinematics(x, y)
        msg = Float64MultiArray()
        msg.data = [shoulder_angle, elbow_angle]
        self.publisher.publish(msg)
        self.get_logger().info("Published Joint Angles: Shoulder = {:.2f}, Elbow = {:.2f}".format(shoulder_angle, elbow_angle))

def main(args=None):
    rclpy.init(args=args)
    ik_solver_node = IKSolverNode()

    # Set the desired end effector position (change this to your target position)
    target_x = 8.0
    target_y = 2.0

    # Calculate and publish the joint angles
    ik_solver_node.publish_joint_angles(target_x, target_y)

    rclpy.spin(ik_solver_node)
    ik_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
