#!/usr/bin/env python
import numpy as np
from math import atan, sqrt, sin, acos, cos, fabs
from numpy import sign

# Controller command interpretation frequency – f = 100 Hz
f = 100
# Maximum joint velocity – 1 rad/s
w_max = 1
# Maximum Cartesian velocity – 1 m/s
v_max = 1
# Maximum joint acceleration – 1 rad/s2
e_max = 1
# Maximum Cartesian acceleration – 1 m/s2
a_max = 1
# Trajectory junction parameter– 5/f
junction = 5

# model of the scara robot
model = {
    'links': [0.6, 0.9, 0.9, 0.3],
    'joints': [0.0, 0.0, 0.0]
}

T_x = lambda d: np.array([[1, 0, 0, d],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
T_y = lambda d: np.array([[1, 0, 0, 0],
                          [0, 1, 0, d],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
T_z = lambda d: np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, d],
                          [0, 0, 0, 1]])
R_x = lambda aplha: np.array([[1.0, 0.0, 0.0, 0.0],
                              [0.0, cos(aplha), -sin(aplha), 0.0],
                              [0.0, sin(aplha), cos(aplha), 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
R_y = lambda aplha: np.array([[cos(aplha), 0.0, sin(aplha), 0.0],
                              [0.0, 1.0, 0.0, 0.0],
                              [-sin(aplha), 0.0, cos(aplha), 0.0],
                              [0.0, 0.0, 0.0, 1.0]])
R_z = lambda aplha: np.array([[cos(aplha), -sin(aplha), 0.0, 0.0],
                              [sin(aplha), cos(aplha), 0.0, 0.0],
                              [0.0, 0.0, 1.0, 0.0],
                              [0.0, 0.0, 0.0, 1.0]])


# todo Calculate Jacobian (skew theory or numeric method)
# todo implement ptp w/o junction
# todo implement lin w/o junction
# todo implement arc w/o junction
# todo Trajectory planning for the following commands: PTP - LIN - ARC - PTP – PTP
# todo implement ptp with junction
# todo implement lin with junction
# todo implement arc with junction
# todo Trajectory planning for the following commands: PTP - LIN - ARC - PTP – PTP with junction
# todo optional Visualization in Gazebo


# todo Report:
# todo Position, Velocity, and Acceleration plots
# todo Link to the project on github.com

def fk(q):
    global model
    links = model['links']
    T = np.dot(np.dot(np.dot(np.dot(np.dot(
        T_z(links[0]), R_z(q[0])),
        T_x(links[1])), R_z(q[1])),
        T_x(links[2])), T_z(-links[3] + q[2]))
    p = np.round(T[:3, 3], 2)
    # print(p)
    return p


def ik(p):
    global model
    links = model['links']
    q = [0, 0, 0]

    d = sqrt(p[0] ** 2 + p[1] ** 2)
    val = round((links[2] ** 2 - links[1] ** 2 - d ** 2) / (2 * links[1] * d), 3)
    val = val if abs(val) < 1.000 else 1.000 * sign(val)
    phi1 = acos(val) % 3.14
    phi2 = atan(p[1] / p[0]) % 3.14 if p[0] != 0 else 0

    q[0] = (phi1 + phi2) % 3.14
    val = round((d ** 2 - links[1] ** 2 - links[2] ** 2) / (2 * links[1] * links[2]), 3)
    val = val if abs(val) < 1.000 else 1.000 * sign(val)
    q[1] = acos(val) % 3.14
    q[2] = p[2] + links[3] - links[0]

    q = np.round(q, 2)
    return q


def jacobian(q):
    global model
    links = model['links']

    # calculating jacobian
    mat_O = []
    mat_O.append(np.array([0,0, links[0]]))
    mat_O.append(np.array([links[1]*cos(q[0]),links[1]*sin(q[0]), links[0]]))
    mat_O.append(np.array([links[1]*cos(q[0]) + links[1]*cos(q[0]+q[1]),
                           links[1]*sin(q[0]) + links[1]*sin(q[0] + q[1]),
                           links[0]]))
    mat_O.append(np.array([links[1]*cos(q[0]) + links[1]*cos(q[0]+q[1]),
                           links[1]*sin(q[0]) + links[1]*sin(q[0] + q[1]),
                           links[0] + q[2] - links[3]]))
    z = []
    z.append(np.array([0,0,1]))
    z.append(np.array([0,0,1]))
    z.append(np.array([0,0,1]))

    J = []

    # 1st joint
    J_i = np.append(np.cross(z[0],mat_O[3]-mat_O[0]), z[0])
    J.append(J_i)

    # 2nd joint
    J_i = np.append(np.cross(z[1],mat_O[3]-mat_O[1]), z[1])
    J.append(J_i)

    # 3rd joint
    J_i = np.append(z[2], np.array([0,0,0]))
    J.append(J_i)

    # converting to numpy array
    J = np.array(J).T
    return J

def ptp(p_0, p_f, j=0):
    pass


def lin():
    pass


def arc():
    pass


if __name__ == '__main__':
    # print(fk([0.7, 0, 0.1]))
    # print(fk([0.7, 0.4, 0.1]))
    # print(ik(fk([0.7, 0, 0.1])))
    # print(ik(fk([0.7, 0.4, 0.1])))
    jacobian([0.7, 0, 0.1])
    # ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
