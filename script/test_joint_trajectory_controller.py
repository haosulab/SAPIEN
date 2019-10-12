import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    client = actionlib.ActionClient('/sapien/movo/right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()
    print 'Connected'

    goal = FollowJointTrajectoryGoal()
    p1 = JointTrajectoryPoint()
    p2 = JointTrajectoryPoint()
    p1.positions = [0] * 10
    p2.positions = [1] * 10
    p1.time_from_start = rospy.Duration.from_sec(0)
    p2.time_from_start = rospy.Duration.from_sec(5)
    goal.trajectory.points = [p1, p2]
    goal.trajectory.joint_names = ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_arm_half_joint',
                                   'right_elbow_joint', 'right_wrist_spherical_1_joint',
                                   'right_wrist_spherical_2_joint', 'right_wrist_3_joint'
                                   ]

    client.send_goal(goal)
    # print res


if __name__ == '__main__':
    rospy.init_node('test_joint_controller')
    main()
