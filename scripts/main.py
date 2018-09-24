#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import  JointControllerState

q1 = {}
q2 = {}
q3 = {}

def callback_q1(data):
    global q1
    q1 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}
    # rospy.loginfo("q1. %s", q1)

def callback_q2(data):
    global q2
    q2 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}
    # rospy.loginfo("q2. %s", q2)

def callback_q3(data):
    global q3
    q3 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}
    # rospy.loginfo("q3. %s", q3)




if __name__ == '__main__':

    rospy.init_node('scara_moving', anonymous=True)

    rospy.Subscriber("/scara/base_to_first_joint_position_controller/state", JointControllerState, callback_q1)
    rospy.Subscriber("/scara/first_to_second_joint_position_controller/state", JointControllerState, callback_q2)
    rospy.Subscriber("/scara/second_to_third_joint_position_controller/state", JointControllerState, callback_q3)

    pub_q1 = rospy.Publisher('/scara/base_to_first_joint_position_controller/command', Float64, queue_size=10)
    pub_q2 = rospy.Publisher('/scara/first_to_second_joint_position_controller/command', Float64, queue_size=10)
    pub_q3 = rospy.Publisher('/scara/second_to_third_joint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)
    # reset poses
    f = 0.0
    while q1.get("set_point") != f:
        print("q1. %s", q1.get("set_point"))
        # rospy.loginfo("Publish: %i", f)
        pub_q1.publish(f)
        rospy.sleep(1)

    while q2.get("set_point") != f:
        print("q2. %s", q2.get("set_point"))
        # rospy.loginfo("Publish: %i", f)
        pub_q2.publish(f)
        rospy.sleep(1)
    f = -0.2
    while q3.get("set_point") != f:
        print("q3. %s", q3.get("set_point"))
        # rospy.loginfo("Publish: %i", f)
        pub_q3.publish(f)
        rospy.sleep(1)

    sum_err = q1.get("error") + q2.get("error") + q2.get("error") - 0.001
    # rospy.loginfo("Sum of errors: %f", sum_err)

    while sum_err  > 0:
        rospy.sleep(0.1)
        sum_err = q1.get("error") + q2.get("error") + q2.get("error") - 0.001
        # rospy.loginfo("0. Sum of errors: %s", sum_err)


    # allready in the initial position




