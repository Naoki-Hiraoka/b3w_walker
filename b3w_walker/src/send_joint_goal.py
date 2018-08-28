#!/usr/bin/env python


import rospy
import argparse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from math import pi

class Sender(object):
    def __init__(self, reset=False):
        self._done = False
        self._reset = reset
        self._rate = rospy.Rate(5.0)  # Hz
        self._threshold = 0.3  # radians
        self._joint_moves = {
            'reset': [0, 0, 0, 0, 0, 0, 0.0, 0.1, 0.1, 0, 0, -0.1, -0.1, 0]
        }
        self._state = 'none'
        self._pub = rospy.Publisher('whole_body_controller/command', JointTrajectory, queue_size=1)
        self._sub = rospy.Subscriber('whole_body_controller/state', JointTrajectoryControllerState, self._check_state)
        self._controller_active = False

    def _check_state(self, msg):
        diff_check = lambda a, b: abs(a - b) <= self._threshold
        angles = msg.actual.positions
        reset_goal = map(diff_check, angles, self._joint_moves['reset'])

        if all(reset_goal):
            self._state = 'reset'
        else:
            self._state = 'none'

        self._controller_active = True
    def _prepare(self):
        while not self._controller_active:
            self._rate.sleep()

    def _move_to(self, goal):
        traj = JointTrajectory()
        traj.joint_names  = [
            'r_shoulder_pitch',
            'r_shoulder_roll',
            'r_elbow_roll',
            'l_shoulder_pitch',
            'l_shoulder_roll',
            'l_elbow_roll',
            'r_footjoint_roll',
            'r_footjoint_pitch',
            'r_ankle_pitch',
            'r_ankle_roll',
            'l_footjoint_roll',
            'l_footjoint_pitch',
            'l_ankle_pitch',
            'l_ankle_roll'
        ]
        traj.points.append(JointTrajectoryPoint())
        traj.points[0].positions = self._joint_moves[goal]
        traj.points[0].time_from_start = rospy.Duration(3)

        while self._state != goal:
            self._pub.publish(traj)
            self._rate.sleep()

    def action(self):
        # Update our starting state to check if arms are tucked
        self._prepare()
        # reset
        if self._reset == True:
            # If arms are already tucked, report this to user and exit.
            # if self._state == 'reset':
            #     rospy.loginfo("reset: already in 'reset' position.")
            # else:
            #     rospy.loginfo("reset: arm is not in 'reset' position.")
            #     self._move_to('reset')
            self._move_to('reset')
            self._done = True
            return

        # Untuck Arms
        else:
            return

    def clean_shutdown(self):
        if not self._done:
            rospy.logwarn('Aborting: Shutting down safely...')
    

def main():
    parser = argparse.ArgumentParser()
    tuck_group = parser.add_mutually_exclusive_group(required=True)
    tuck_group.add_argument("-r", "--reset", dest="reset",
        action='store_true', default=False, help="reset pose")
    args = parser.parse_args(rospy.myargv()[1:])
    reset = args.reset
    print reset
    rospy.loginfo("Initializing node... ")
    rospy.init_node("send_joint_goal")
    sender = Sender(reset=reset)
    rospy.on_shutdown(sender.clean_shutdown)
    sender.action()
    rospy.loginfo("Finished" )

if __name__ == "__main__":
    main()
