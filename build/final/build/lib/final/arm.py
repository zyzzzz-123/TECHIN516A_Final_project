import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import time
import rclpy
from geometry_msgs.msg import Pose, Point, Quaternion


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

    gripper.move_to_position(0.0)

    gripper.close()
    poses = load_poses_from_csv("/home/yuzhez23@netid.washington.edu/ros2_ws/src/final/final/final.csv")


    move(arm,poses["point1"])
    move(arm,poses["point2"])
    gripper.move_to_position(0.7)
    move(arm,poses["point3"])
    arm.go_vertical()
    move(arm,poses["point4"])
    gripper.move_to_position(0.0)

    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()
