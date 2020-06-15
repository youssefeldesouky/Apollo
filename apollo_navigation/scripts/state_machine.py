#!/usr/bin/env python
import rospy
import smach
import smach_ros
from move_base_msgs.msg import *
from std_msgs.msg import Float64
from control_msgs.msg import JointControllerState
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
import argparse

# raisebeforegoingback 0
# raisetheproduct 1
# lowerthefork 2

class Fork(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["failed", "succeeded", "drop", "end"], input_keys=["state", "goal", "target", "free"], output_keys=["direction", "target"])
        self.pub = rospy.Publisher("fork_position_controller/command", Float64, queue_size=1)
        self.sub = 0
        self.err = 0
        self.goal = 0
        print("3mad niggaronni")

    def execute(self, ud):
        if ud.free:
            return "end"
        if ud.state == 0:
            self.goal = ud.goal
        elif ud.state == 1:
            self.goal += 0.02
        elif ud.state == 2:
            self.goal = 0.02

        self.pub.publish(self.goal)
        self.sub = rospy.Subscriber("fork_position_controller/state", JointControllerState, self.fork_cb)
        while self.err > 0.0001:
            pass
        rospy.sleep(1)
        if ud.state == 0:
            ud.direction = -1
            print("micah el nigga 0")
        elif ud.state == 1:
            ud.direction = 1
            print("joe el nigga 2")

        if ud.state == 2:
            ud.target = PoseStamped()
            ud.target.header.frame_id = "map"
            ud.target.pose.position.x = 2.0  # args
            ud.target.pose.position.y = 1.0  # args
            ud.target.pose.orientation.w = 1.0  # args
            return "drop"
        else:
            return "succeeded"

    def fork_cb(self, data):
        self.err = data.error
        if self.err < 0.0001:
            self.sub.unregister()


class Linear(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["failed", "succeeded"], input_keys=["direction", "state"], output_keys=["state"])
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(50)
        self.sub = 0
        self.range = 0
        print("shit")


    def execute(self, ud):
        t = Twist()
        t.linear.x = ud.direction * 0.3
        self.sub = rospy.Subscriber("rear_ultrasonic", Range, self.us_cb)
        print(ud.direction)
        rospy.sleep(1)
        if ud.direction < 0:
            while self.range >= 0.05:
                self.pub.publish(t)
                self.rate.sleep()
        elif ud.direction > 0:
            while self.range <= 0.2:
                self.pub.publish(t)
                self.rate.sleep()

        if ud.state == 0:
            ud.state = 1
        elif ud.state == 1:
            ud.state = 2

        return "succeeded"

    def us_cb(self, data):
        self.range = data.range


def result_cb(ud, status, result):
    if status == 3:
        print("\t\t\tniggaroni\n")


def main():
    rospy.init_node("nav_state_machine", anonymous=True)
    sm = smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
    sm.userdata.target_pose = PoseStamped()
    sm.userdata.target_pose.header.frame_id = "map"
    sm.userdata.target_pose.pose.position.x = 1.0  # args
    sm.userdata.target_pose.pose.position.y = 1.0  # args
    sm.userdata.target_pose.pose.orientation.w = 1.0  # args
    sm.userdata.fork_level = 0.15
    sm.userdata.state = 0
    sm.userdata.emad = False

    with sm:
        smach.StateMachine.add("MOVE_BASE",
                               smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_slots=["target_pose"],
                                                           result_cb=result_cb),
                               transitions={"succeeded": "FORK", "aborted": "aborted", "preempted": "preempted"},
                               remapping={"target_pose": "target_pose"})
        smach.StateMachine.add("FORK", Fork(), transitions={"succeeded": "LIN", "failed": "aborted", "drop": "MOVE_BASE", "end": "succeeded"},
                               remapping={"state": "state", "goal": "fork_level", "direction": "direction", "target": "target_pose", "free": "emad"})
        smach.StateMachine.add("LIN", Linear(), transitions={"succeeded": "FORK", "failed": "aborted"},
                               remapping={"direction": "direction", "state": "state"})

    sis = smach_ros.IntrospectionServer("move_base", sm, "SM_ROOT")
    sis.start()

    sm.execute()

    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()