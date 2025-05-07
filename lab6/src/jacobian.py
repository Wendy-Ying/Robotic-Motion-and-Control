#!/usr/bin/env python3
import math
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# Class representing a single joint/link of the robotic arm
class Link:
    def __init__(self, dh_params):
        # Store DH parameters for the link: [alpha, a, d, theta_offset]
        self.dh_params_ = dh_params
    
    def transformation_matrix(self, theta):
        # Compute the transformation matrix for this link (based on DH parameters)
        alpha = self.dh_params_[0]
        a = self.dh_params_[1]
        d = self.dh_params_[2]
        theta = theta + self.dh_params_[3]  # Add fixed offset angle
        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        
        # Return the transformation matrix
        trans = np.array([[ct, -st, 0, a],
                          [st*ca, ct*ca, -sa, -sa*d],
                          [st*sa, ct*sa, ca, ca*d],
                          [0, 0, 0, 1]])
        return trans

    @staticmethod
    def basic_jacobian(trans, ee_pos):
        # Compute the Jacobian row for this joint based on spatial z-axis and position
        pos = np.array([trans[0, 3], trans[1, 3], trans[2, 3]])  # Joint position
        z_axis = np.array([trans[0, 2], trans[1, 2], trans[2, 2]])  # z-axis of this joint

        # Compute linear velocity part (z × (end-effector - joint pos)) and angular velocity (z-axis itself)
        basic_jacobian = np.hstack((np.cross(z_axis, ee_pos - pos), z_axis))
        return basic_jacobian

# Class representing a multi-joint robotic arm
class NLinkArm:
    def __init__(self, dh_params_list) -> None:
        self.link_list = []
        for i in range(len(dh_params_list)):
            self.link_list.append(Link(dh_params_list[i]))  # Construct Link objects one by one

    def transformation_matrix(self, thetas):
        # Compute full forward kinematics transformation matrix of the arm
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix(thetas[i]))
        return trans

    def forward_kinematics(self, thetas):
        # Compute end-effector pose (position + Euler angles)
        trans = self.transformation_matrix(thetas)
        x, y, z = trans[0, 3], trans[1, 3], trans[2, 3]
        alpha, beta, gamma = self.euler_angle(thetas)
        return [x, y, z, alpha, beta, gamma]

    def euler_angle(self, thetas):
        # Extract Euler angles (ZYZ convention) from the transformation matrix
        trans = self.transformation_matrix(thetas)

        alpha = math.atan2(trans[1][2], trans[0][2])
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) + math.pi
        if not (-math.pi / 2 <= alpha <= math.pi / 2):
            alpha = math.atan2(trans[1][2], trans[0][2]) - math.pi

        beta = math.atan2(
            trans[0][2]*math.cos(alpha) + trans[1][2]*math.sin(alpha),
            trans[2][2])
        gamma = math.atan2(
            -trans[0][0]*math.sin(alpha) + trans[1][0]*math.cos(alpha),
            -trans[0][1]*math.sin(alpha) + trans[1][1]*math.cos(alpha))

        return alpha, beta, gamma

    def inverse_kinematics(self, ref_ee_pose):
        # Use numerical (iterative) method to compute inverse kinematics
        # Try to make current end-effector pose approach the target pose
        thetas = [0, 0, 0, 0, 0, 0]
        for cnt in range(500):
            ee_pose = self.forward_kinematics(thetas)
            diff_pose = np.array(ref_ee_pose) - ee_pose  # Difference between current and desired pose

            basic_jacobian_mat = self.basic_jacobian(thetas)
            alpha, beta, gamma = self.euler_angle(thetas)

            # Construct ZYZ Euler angle transformation matrix
            K_zyz = np.array([
                [0, -math.sin(alpha), math.cos(alpha)*math.sin(beta)],
                [0,  math.cos(alpha), math.sin(alpha)*math.sin(beta)],
                [1, 0, math.cos(beta)]
            ])
            K_alpha = np.identity(6)
            K_alpha[3:, 3:] = K_zyz

            # Iteratively update joint angles
            theta_dot = np.dot(np.dot(np.linalg.pinv(basic_jacobian_mat), K_alpha), diff_pose)
            thetas = thetas + theta_dot / 100.
        return thetas

    def basic_jacobian(self, thetas):
        # Construct the full Jacobian matrix for the arm
        ee_pos = self.forward_kinematics(thetas)[0:3]  # Get end-effector position
        basic_jacobian_mat = []
        trans = np.identity(4)
        for i in range(len(self.link_list)):
            trans = np.dot(trans, self.link_list[i].transformation_matrix(thetas[i]))
            basic_jacobian_mat.append(self.link_list[i].basic_jacobian(trans, ee_pos))
        return np.array(basic_jacobian_mat).T  # Transpose to get 6xN matrix

# Main program entry point
if __name__ == "__main__":
    rospy.init_node("jacobian_test")  # Initialize ROS node
    tool_pose_pub = rospy.Publisher("/tool_pose_cartesian", Point, queue_size=1)
    tool_velocity_pub = rospy.Publisher("/tool_velocity_cartesian", Point, queue_size=1)
    tool_force_pub = rospy.Publisher("/tool_force_cartesian", Point, queue_size=1)

    # Define DH parameters for the arm: [alpha, a, d, theta_offset]
    dh_params_list = np.array([
        [0, 0, 243.3/1000, 0],
        [math.pi/2, 0, 10/1000, math.pi/2],
        [math.pi, 280/1000, 0, math.pi/2],
        [math.pi/2, 0, 245/1000, math.pi/2],
        [math.pi/2, 0, 57/1000, 0],
        [-math.pi/2, 0, 235/1000, -math.pi/2]
    ])
    gen3_lite = NLinkArm(dh_params_list)

    # ROS loop: continuously listen to joint states and publish end-effector data
    while not rospy.is_shutdown():
        feedback = rospy.wait_for_message("/my_gen3_lite/joint_states", JointState)
        thetas = feedback.position[0:6]
        velocities = feedback.velocity[0:6]
        torques = feedback.effort[0:6]

        # Compute end-effector pose, velocity, and force
        tool_pose = gen3_lite.forward_kinematics(thetas)
        J = gen3_lite.basic_jacobian(thetas)
        tool_velocity = J.dot(velocities)
        tool_force = np.linalg.pinv(J.T).dot(torques)

        # Pack into ROS message format
        tool_pose_msg = Point(x=tool_pose[0], y=tool_pose[1], z=tool_pose[2])
        tool_velocity_msg = Point(x=tool_velocity[0], y=tool_velocity[1], z=tool_velocity[2])
        tool_force_msg = Point(x=tool_force[0], y=tool_force[1], z=tool_force[2])

        # Publish to ROS topics
        tool_pose_pub.publish(tool_pose_msg)
        tool_velocity_pub.publish(tool_velocity_msg)
        tool_force_pub.publish(tool_force_msg)

        # Print debug information
        print(f"joint position: {thetas}")
        print(f"joint velocity: {velocities}")
        print(f"joint torque: {torques}")
        print(f"tool position: {tool_pose}")
        print(f"tool velocity: {tool_velocity}")
        print(f"tool torque: {tool_force}")
