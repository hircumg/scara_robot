#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from control_msgs.msg import  JointControllerState
from math import sqrt,acos, atan
from numpy import sign

q1 = {}
q2 = {}
q3 = {}



def callback_q1(data):
    global q1
    # save only neadble elements from topic
    q1 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}

def callback_q2(data):
    global q2
    # save only neadble elements from topic
    q2 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}

def callback_q3(data):
    global q3
    # save only neadble elements from topic
    q3 = {'error' : data.error, 'process_value' : data.process_value, 'set_point' : data.set_point}


def create_points_xy(k = 100):
    points = []
    r = 0.5
    z = 0.1  # It will be a constant during rotation

    # function for calculating circle
    f = lambda x: sqrt(r**2 - (x - 1.3)**2) if x - 0.8 > 0 else 0.0

    # add first point of the circle
    x = 1.8
    y = f(x)
    points.append([x, y, z])

    # determine step for changing x
    step = 0.1
    step = r / k

    # generane first half of points
    for i in range(0,2*k):
        x = x - step
        y =f(x)
        points.append([x,y,z])

    # generate second half of points
    for i in range(0,2*k):
        x = x + step
        y =-f(x)
        points.append([x, y, z])

    # revers the order of points to rotate in counter clockwise
    points.reverse()
    return points

def create_points_yz(k = 100):

    points = []
    r = 0.3
    x = 0.8 # It will be a constant during rotation

    # function for calculating circle
    f = lambda z: sqrt(r**2 - (z-0.3)**2) if abs(r**2 - (z-0.3)**2) >  0.00001 else 0.0

    # add first point of the circle
    z = 0.6
    y = f(z)
    points.append([x,y,z])

    # determine step for changing x
    step = 0.1
    step = r / k

    # generane first half of points
    for i in range(0,2*k):
        z = z - step
        y =f(z)
        points.append([x, y, z])

    # generate second half of points
    for i in range(0,2*k):
        z = z + step
        y =-f(z)
        points.append([x, y, z])

    # revers the order of points to rotate in counter clockwise
    points.reverse()
    return points

def points_to_angles(points):
    angles = []

    for point in points:
        # inverse kinematic for SCARA robot
        d = sqrt(point[0]**2 + point[1]**2)
        l = 0.9
        phi1 = acos(d/(2*l))
        phi2 = atan(point[1]/point[0]) if point[0] != 0 else 0
        q1 = - phi1 + phi2
        q2 = acos((d**2 - 2 * l**2)/(2 * l **2))
        q3 = point[2] - 0.3

        # make sure that angles is not out limits
        q1 = q1 if abs(q1) < 2.6 else 2.6 * sign(q1)
        q2 = q2 if abs(q2) < 2.6 else 2.6 * sign(q2)
        q3 = q3 if abs(q3) < 0.3 else 0.3 * sign(q3)

        angles.append([q1,q2,q3])

    angles.reverse()
    return angles



def set_angles(q1_angle, q2_angle, q3_angle = -0.2):
    global q1, q2, q3, pub_q1, pub_q2, pub_q3
    # the error of each point
    err = 0.01

    # publish angles to the topics
    while (q1.get("set_point") != q1_angle) or (q2.get("set_point") != q2_angle) or (q3.get("set_point") != q3_angle):
        pub_q1.publish(q1_angle)
        pub_q2.publish(q2_angle)
        pub_q3.publish(q3_angle)
        rospy.sleep(0.01)



    # rotate till manipulator will be in the right position with some 'err'
    q1_err = abs(q1.get("error")) - err
    q2_err = abs(q2.get("error")) - err
    q3_err = abs(q3.get("error")) - err
    while (q1_err > 0) or (q2_err > 0) or (q3_err > 0):
        rospy.sleep(0.01)
        q1_err = abs(q1.get("error")) - err
        q2_err = abs(q2.get("error")) - err
        q3_err = abs(q3.get("error")) - err



def move_XY_circle(precision_factor = 100):
    # calculate points to circle
    points_xy = create_points_xy(k=precision_factor)

    # translate points of the circle into angles
    angles_xy = points_to_angles(points_xy)

    # set angles in the right order
    for angle in angles_xy:
        set_angles(angle[0], angle[1], angle[2])

def move_YZ_circle(precision_factor = 100):
    # calculate points to circle
    points_yz = create_points_yz(k=precision_factor)

    # translate points of the circle into angles
    angles_yz = points_to_angles(points_yz)

    # set angles in the right order
    for angle in angles_yz:
        set_angles(angle[0], angle[1], angle[2])






if __name__ == '__main__':
    global sub_q1, sub_q2, sub_q3, pub_q1, pub_q2, pub_q3


    rospy.init_node('scara_moving', anonymous=True)

    # create subscrivers for reading current joint states
    sub_q1 = rospy.Subscriber("/scara/base_to_first_joint_position_controller/state", JointControllerState, callback_q1)
    sub_q2 = rospy.Subscriber("/scara/first_to_second_joint_position_controller/state", JointControllerState, callback_q2)
    sub_q3 = rospy.Subscriber("/scara/second_to_third_joint_position_controller/state", JointControllerState, callback_q3)

    # create publishers to send angles to joints
    pub_q1 = rospy.Publisher('/scara/base_to_first_joint_position_controller/command', Float64, queue_size=10)
    pub_q2 = rospy.Publisher('/scara/first_to_second_joint_position_controller/command', Float64, queue_size=10)
    pub_q3 = rospy.Publisher('/scara/second_to_third_joint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(100)


    # go to initial point and make circle in YZ plane
    set_angles(-1.11, 2.22, 0.3)
    rospy.sleep(5)
    move_YZ_circle(300)

    # go to initial point and make circle in YZ plane
    set_angles(0.0, 0.0, -0.2)
    rospy.sleep(5)
    move_XY_circle(300)


