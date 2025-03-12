import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion




import time
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion
from pyquaternion import Quaternion as PyQuaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper  
import numpy as np
from rclpy.node import Node

class OdomCloseLoop(Node):
    def __init__(self, direction=1, target_distance=1.1):
        super().__init__('close_loop_odom')
        # debug mode
        self.target_distance = target_distance
        self.debug = True
        self.direction = direction
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # TODO: subscribe to /odom topic
        # Publisher to control the velocity
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # TODO: add the publisher to send velocity values
        self.position_x_list = []
        self.motion_move = Twist()
        self.motion_stop = Twist()
        self.movement_complete = False
        # Set constant speed to move forward
        self.motion_move.linear.x = 0.1 * direction
        # Set speed to stop
        self.motion_stop.linear.x = 0.0

    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)
        # Ensure the list has at least 2 elements
        if len(self.position_x_list) > 1:
            initial_position = self.position_x_list[0]
            traveled_distance = abs(position_x - initial_position)
            # Check if the last recorded position is greater than the first one
            # plus the desired distance to travel
            print(initial_position)
            print(position_x)
            if traveled_distance>= self.target_distance:
            # TODO: fill in the condition to check whether the last item on the list
            # is greater than the first one plus the desired distance to be traveled
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                self.movement_complete = True
            else:
                self.pub.publish(self.motion_move)
                if self.debug:
                    self.get_logger().info(f"Traveled distance: {traveled_distance}")




def load_poses_from_csv(file_path):
    """
    Load pose data from a CSV file and return a dictionary of Pose objects.
    """
    poses = {}
    with open(file_path, "r") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            pose = Pose()
            pose.position = Point(
                x=float(row['pos_x']),
                y=float(row['pos_y']),
                z=float(row['pos_z'])
            )
            pose.orientation = Quaternion(
                x=float(row['ori_x']),
                y=float(row['ori_y']),
                z=float(row['ori_z']),
                w=float(row['ori_w'])
            )
            poses[row['name']] = pose
    return poses

def move(entity, pose=None, delay=1):
    entity.inverse_kinematic_movement(pose)
    time.sleep(delay)

def main():

    rclpy.init()
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    print("starting... ")
    # # run forward first 
    # odom_cl = OdomCloseLoop(target_distance=1.6)
    # while rclpy.ok() and not odom_cl.movement_complete:
    #     rclpy.spin_once(odom_cl)
    # odom_cl.destroy_node()
    # time.sleep(1)
    # print("first part finished.")

    # pick the tube and put it on the turtlebot
    gripper.move_to_position(0.0)
    poses = load_poses_from_csv("/home/yuzhez23@netid.washington.edu/ros2_ws/src/final/final/final.csv")

    move(arm,poses["point1"])
    move(arm,poses["point2"])
    gripper.move_to_position(0.7)
    move(arm,poses["point3"])
    arm.go_vertical()
    move(arm,poses["point4"])
    gripper.move_to_position(0.0)


    # odom_cl = OdomCloseLoop(direction=-1, target_distance = 1.6)
    # while rclpy.ok() and not odom_cl.movement_complete:
    #     rclpy.spin_once(odom_cl)
    # odom_cl.destroy_node()

    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
