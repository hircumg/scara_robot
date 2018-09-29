#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import  JointControllerState
from math import sqrt,acos, atan
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


def create_points_xy(k = 100):

    points = []
    r = 0.5
    f = lambda x: sqrt(r**2 - (x - 1.3)**2) if x - 0.8 > 0 else 0.0
    x = 1.8
    y = f(x)
    rospy.loginfo("x: %f, y: %f", x, y)
    points.append([x,y])
    step = 0.1
    step = r / k
    rospy.loginfo("step: %f", step)

    # generane first half of points
    for i in range(0,2*k):
        x = x - step
        rospy.loginfo('i: %i, x: %f', i, x)
        y =f(x)
        rospy.loginfo('i: %i, x: %f, y: %f', i, x, y)
        points.append([x,y])


    # generate secon half of points
    for i in range(0,2*k):
        x = x + step
        y =-f(x)
        # rospy.loginfo('i: %i, x: %f, y: %f', i, x, y)
        points.append([x,y])

    # revers the order of points to rotate in counter clockwise
    points.reverse()

    return points

def points_to_angles(points):
    angles = []


    for point in points:
        d = sqrt(point[0]**2 + point[1]**2)
        l = 0.9
        phi1 = acos(d/(2*l))
        phi2 = atan(point[1]/point[0]) if point[0] != 0 else 0
        q1 = - phi1 + phi2
        q2 = acos((d**2 - 2 * l**2)/(2 * l **2))

        rospy.loginfo('new angles: %s and d: %f, phi1: %f, phi2: %f', [point[0], point[1], q1, q2], d, phi1, phi2)
        angles.append([point[0], point[1], q1,q2])

    angles.reverse()

    return angles



if __name__ == '__main__':
    global sub_q1, sub_q2, sub_q3, pub_q1, pub_q2, pub_q3

    rospy.init_node('scara_moving', anonymous=True)
    sub_q1 = rospy.Subscriber("/scara/base_to_first_joint_position_controller/state", JointControllerState, callback_q1)
    sub_q2 = rospy.Subscriber("/scara/first_to_second_joint_position_controller/state", JointControllerState, callback_q2)
    sub_q3 = rospy.Subscriber("/scara/second_to_third_joint_position_controller/state", JointControllerState, callback_q3)

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

    f = 0.0
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

    q1_err = abs(q1.get("error")) - 0.01
    q2_err = abs(q2.get("error")) - 0.01
    # rospy.loginfo('cur_angle %s and initial errors: %s', angle, [q1_err, q2_err])

    while (q1_err > 0) or (q2_err > 0):
        rospy.sleep(0.01)
        q1_err = abs(q1.get("error")) - 0.01
        q2_err = abs(q2.get("error")) - 0.01


    # exit(0)
    # close subscribing q3
    sub_q3.unregister()

    points_xy = create_points_xy(k=1000)
    rospy.loginfo('points in main: %s', points_xy)
    angles_xy = points_to_angles(points_xy)
    rospy.loginfo('angles XY: %s', angles_xy)


    for angle in angles_xy:
        rospy.loginfo('cur_angle %s', angle)
        f = angle[2]
        while (q1.get("set_point") != angle[2]) or (q2.get("set_point") != angle[3]):
            print("q1. %s", q1.get("set_point"))
            # rospy.loginfo("Publish: %i", f)
            print("q2. %s", q2.get("set_point"))
            # rospy.loginfo("Publish: %i", f)
            pub_q1.publish(angle[2])
            pub_q2.publish(angle[3])
            rospy.sleep(0.01)

        # sum_err = q1.get("error") + q2.get("error") + q2.get("error") - 0.001
        # # rospy.loginfo("Sum of errors: %f", sum_err)
        q1_err = abs(q1.get("error")) - 0.01
        q2_err = abs(q2.get("error")) - 0.01
        rospy.loginfo('cur_angle %s and initial errors: %s', angle, [q1_err,q2_err])

        while (q1_err > 0) or (q2_err > 0):
            rospy.sleep(0.01)
            q1_err = abs(q1.get("error")) - 0.01
            q2_err = abs(q2.get("error")) - 0.01
            # sum_err = q1.get("error") + q2.get("error") + q2.get("error") - 0.001
            # # rospy.loginfo("0. Sum of errors: %s", sum_err)

        rospy.loginfo('ended errors: %s', [q1_err, q2_err])
        # if(angle[0] == 1.3) and (angle[1] == -0.5):
        #     exit(0)

    # rospy.sleep(10)
    #
    # f = angles_xy[1][2]
    # while q1.get("set_point") != f:
    #     print("q1. %s", q1.get("set_point"))
    #     # rospy.loginfo("Publish: %i", f)
    #     pub_q1.publish(f)
    #     rospy.sleep(1)
    #
    # f = angles_xy[1][3]
    # while q2.get("set_point") != f:
    #     print("q2. %s", q2.get("set_point"))
    #     # rospy.loginfo("Publish: %i", f)
    #     pub_q2.publish(f)
    #     rospy.sleep(1)




