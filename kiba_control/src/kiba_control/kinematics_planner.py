from threading import Lock

import rospy
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped
from kdl_parser_py.urdf import treeFromParam
from sensor_msgs.msg import JointState
import tf2_geometry_msgs # Needed to get automatic type conversions for TF2, even if PyCharm complains ;)
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
import PyKDL


class KinematicsPlanner(object):
    def __init__(self, action_name, timeout):
        """
        Constructor for KinematicsPlanner class.
        :param action_name: Name of the action of trajectory controller.
        :type action_name: str
        :param timeout: Timeout to wait for the action server of the controller to show up.
        :type timeout: rospy.rostime.Duration
        """
        self.lock = Lock()

        self.root_frame_name = rospy.get_param('~root_frame_name')
        self.tip_frame_name = rospy.get_param('~tip_frame_name')

        self.robot_model = URDF.from_parameter_server() # type: URDF
        (success, tree) = treeFromParam('/robot_description') # type: PyKDL.Tree
        if success:
            self.chain = tree.getChain(self.root_frame_name, self.tip_frame_name) # type: PyKDL.Chain
        else:
            raise RuntimeError("Reading of robot URDF failed.")

        self.joint_state = {}
        self.js_sub = rospy.Subscriber('~joint_states', JointState, self.js_callback)
        self.goal_sub = rospy.Subscriber('~goal', PoseStamped, self.goal_callback)
        self.action_client = SimpleActionClient(action_name, FollowJointTrajectoryAction)
        if not self.action_client.wait_for_server(timeout):
            raise RuntimeError("Waiting for action server '{}' timed out.".format(action_name))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

    def js_callback(self, msg):
        """
        Callback function for subscription to new joint state measurements.
        :param msg: Current joint state message of the robot.
        :type msg: JointState
        :return: Nothing.
        """
        with self.lock:
            self.joint_state.clear()
            for idx, joint_name in enumerate(msg.name):
                self.joint_state[joint_name] = msg.position[idx]

    def goal_callback(self, msg):
        """
        Callback function for subscription to new desired end-effector poses of the robot.
        :param msg: New desired end-effector pose for the robot.
        :type msg: PoseStamped
        :return: Nothing.
        """
        with self.lock:
            try:
                self.action_client.cancel_all_goals()
                self.action_client.send_goal(self.get_trajectory(msg))
            except RuntimeError as e:
                rospy.logerr('{}'.format(e))

    def get_trajectory(self, goal):
        """
        Calculates and returns a sparse joint trajectory to reach a desired goal pose for the end-effector of the robot.
        :param goal: New desired end-effector pose for the robot.
        :type goal: PoseStamped
        :return: Spare joint trajectory that moves the robot to the desired goal.
        :rtype: FollowJointTrajectoryGoal
        """
        result = FollowJointTrajectoryGoal()
        result.trajectory.header = goal.header
        result.trajectory.joint_names = self.get_joint_names()
        result.trajectory.points.append(self.get_trajectory_start())
        result.trajectory.points.append(self.get_trajectory_end(goal))
        return result

    def get_joint_names(self):
        """
        Calculates and returns the joint names in the robot's kinematic chain.
        :return: Joint names in the robot's kinematic chain.
        :rtype: list(str)
        """
        return self.joint_state.keys()

    def get_dof(self):
        """
        Calculates and returns the degrees of freedom (DOF) of the controlled robot.
        :return: The DOF of the controlled robot.
        :rtype: int
        """
        return len(self.get_joint_names())

    def get_chain_joint_names(self):
        """
        Calculates and returns the joint names of the robot, in the order expected by KDL utils.
        :return: Joint names of the robot, in the order expected by KDL utils.
        :rtype: list(str)
        """
        joint_names = []
        for idx in range(0, self.chain.getNrOfSegments()):
            joint_name = self.chain.getSegment(idx).getJoint().getName()
            if self.is_movable_joint(joint_name):
                joint_names.append(joint_name)
        return joint_names

    def get_limits(self):
        """
        Calculates and returns the lower and upper joint limits of the controlled robot, in the order expected by KDL.
        :return: The lower and upper joint limits of the controlled robot, in the order expected by KDL.
        :rtype: tuple(PyKDL.JntArray, PyKDL.JntArray)
        """
        lower = PyKDL.JntArray(self.chain.getNrOfSegments())
        upper = PyKDL.JntArray(self.chain.getNrOfSegments())
        for idx, joint_name in enumerate(self.get_chain_joint_names()):
            if self.is_joint_with_limits(joint_name):
                lower[idx] = self.robot_model.joint_map[joint_name].limit.lower
                upper[idx] = self.robot_model.joint_map[joint_name].limit.upper
            else:
                lower[idx] = -10e10
                upper[idx] = +10e10
        return (lower, upper)

    def is_joint_with_limits(self, joint_name):
        """
        Checks whether a particular robot joint has limits.
        :param joint_name: Name of the joint to check.
        :return: True if the joint has limits, else False.
        :rtype: bool
        """
        return self.robot_model.joint_map[joint_name].type in ('revolute', 'prismatic')

    def is_movable_joint(self, joint_name):
        """
        Checks whether a particular robot joint is mobile.
        :param joint_name: Name of the joint to check.
        :return: True if the joint is mobile, else False.
        :rtype: bool
        """
        return self.robot_model.joint_map[joint_name].type in ('revolute', 'prismatic', 'continuous')

    def get_trajectory_start(self):
        """
        Calculates and returns the start point of the joint trajectory to the desired goal.
        :return: The start point of the joint trajectory.
        :rtype: JointTrajectoryPoint
        """
        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(0.1)
        p.positions = self.joint_state.values()
        return p

    def get_trajectory_end(self, msg):
        """
        Calculates and return the end point of the joint trajectory that moves the robot to a desired end-effector pose.
        :param msg: Desired pose for the robot's end-effector.
        :type msg: PoseStamped
        :return: The end point of the joint trajectory that moves the robot to the desired end-effector pose.
        :rtype: JointTrajectoryPoint
        """
        p = JointTrajectoryPoint()
        p.time_from_start = rospy.Duration(2.0)

        # set up all KDL solvers that we need to calculate the end point
        fk_solver_pos = PyKDL.ChainFkSolverPos_recursive(self.chain)
        ik_solver_vel = PyKDL.ChainIkSolverVel_wdls(self.chain)
        (lower_limits, upper_limits) = self.get_limits()
        ik_solver_pos = PyKDL.ChainIkSolverPos_NR_JL(self.chain, lower_limits, upper_limits, fk_solver_pos, ik_solver_vel)

        # convert current joint state to KDL format
        current_joint_state = PyKDL.JntArray(self.get_dof())
        for idx, joint_name in enumerate(self.get_chain_joint_names()):
            current_joint_state[idx] = self.joint_state[joint_name]

        # transform goal Pose first into root frame of robot and then into KDL.Frame for the solver
        goal_in_root = self.tf_buffer.transform(msg, self.root_frame_name)
        goal_in_kdl = PyKDL.Frame(PyKDL.Rotation.Quaternion(
            goal_in_root.pose.orientation.x, goal_in_root.pose.orientation.y,
            goal_in_root.pose.orientation.z, goal_in_root.pose.orientation.w),
            PyKDL.Vector(goal_in_root.pose.position.x, goal_in_root.pose.position.y, goal_in_root.pose.position.z))

        # busy the solver to compute the IK solution
        goal_joint_state = PyKDL.JntArray(current_joint_state)
        if ik_solver_pos.CartToJnt(current_joint_state, goal_in_kdl, goal_joint_state) < 0:
            raise RuntimeError("IK Solver did not find a solution.")

        # copy result in the correct order into the TrajectoryPoint
        aux_goal_map = {joint_name: joint_pos for joint_name, joint_pos in
                        zip(self.get_chain_joint_names(), goal_joint_state)}
        for joint_name in self.get_joint_names():
            p.positions.append(aux_goal_map[joint_name])

        return p

