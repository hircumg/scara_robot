#!/usr/bin/env python
import numpy as np
from math import atan, sqrt, sin, acos, cos, fabs
from numpy import sign
from matplotlib import pyplot as plt

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
    mat_O.append(np.array([0, 0, links[0]]))
    mat_O.append(np.array([links[1] * cos(q[0]), links[1] * sin(q[0]), links[0]]))
    mat_O.append(np.array([links[1] * cos(q[0]) + links[1] * cos(q[0] + q[1]),
                           links[1] * sin(q[0]) + links[1] * sin(q[0] + q[1]),
                           links[0]]))
    mat_O.append(np.array([links[1] * cos(q[0]) + links[1] * cos(q[0] + q[1]),
                           links[1] * sin(q[0]) + links[1] * sin(q[0] + q[1]),
                           links[0] + q[2] - links[3]]))
    z = []
    z.append(np.array([0, 0, 1]))
    z.append(np.array([0, 0, 1]))
    z.append(np.array([0, 0, 1]))

    J = []

    # 1st joint
    J_i = np.append(np.cross(z[0], mat_O[3] - mat_O[0]), z[0])
    J.append(J_i)

    # 2nd joint
    J_i = np.append(np.cross(z[1], mat_O[3] - mat_O[1]), z[1])
    J.append(J_i)

    # 3rd joint
    J_i = np.append(z[2], np.array([0, 0, 0]))
    J.append(J_i)

    # converting to numpy array
    J = np.array(J).T
    return J


def ptp(p_0, p_f, j=0):
    global w_max, f, e_max
    T = 1 / f
    q_0 = ik(p_0)
    q_f = ik(p_f)
    print(q_0, q_f)
    times = []
    times.append({'b': (w_max / e_max), 'p': (abs(q_f[0] - q_0[0]) / w_max - (w_max / e_max)),
                  'b_triangle': (sqrt(abs(q_f[0] - q_0[0]) / e_max))})
    times.append({'b': (w_max / e_max), 'p': (abs(q_f[1] - q_0[1]) / w_max - (w_max / e_max)),
                  'b_triangle': (sqrt(abs(q_f[1] - q_0[1]) / e_max))})
    times.append({'b': (w_max / e_max), 'p': (abs(q_f[2] - q_0[2]) / w_max - (w_max / e_max)),
                  'b_triangle': (sqrt(abs(q_f[2] - q_0[2]) / e_max))})
    print(times)

    max_plato = 0
    for time in times:
        if time['p'] > max_plato:
            max_plato = time['p']

    max_t_b = 0
    for time in times:
        t_b = time['b'] if time['b'] < time['b_triangle'] else time['b_triangle']
        if t_b > max_t_b:
            max_t_b = t_b

    # provide changing params according to controller command interpretation frequency
    max_plato = (max_plato // T + (round(max_plato % T, 2) != 0)) * T
    max_t_b = (max_t_b // T + (round(max_t_b % T, 2) != 0)) * T
    print("t_b: %.2f, t_plato: %.2f" % (max_t_b, max_plato))

    joints_params = []
    joints_params.append({'e': (abs(q_f[0] - q_0[0]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': (abs(q_f[0] - q_0[0]) / (max_plato + max_t_b))})
    joints_params.append({'e': (abs(q_f[1] - q_0[1]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': (abs(q_f[1] - q_0[1]) / (max_plato + max_t_b))})
    joints_params.append({'e': (abs(q_f[2] - q_0[2]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': (abs(q_f[2] - q_0[2]) / (max_plato + max_t_b))})
    print(joints_params)

    joint_values = {'q0': [], 'q1': [], 'q2': []}
    prev_ang_vel = [0, 0, 0]
    prev_ang_pos = [0, 0, 0]

    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] + joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] + joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] + joints_params[2]['e'] * T

        joint_values['q0'].append([i * T, joints_params[0]['e'], prev_ang_vel[0], prev_ang_pos[0]])
        joint_values['q1'].append([i * T, joints_params[1]['e'], prev_ang_vel[1], prev_ang_pos[1]])
        joint_values['q2'].append([i * T, joints_params[2]['e'], prev_ang_vel[2], prev_ang_pos[2]])

    for i in range(int(max_plato / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + joints_params[0]['w'] * T
        prev_ang_pos[1] = prev_ang_pos[1] + joints_params[1]['w'] * T
        prev_ang_pos[2] = prev_ang_pos[2] + joints_params[2]['w'] * T
        joint_values['q0'].append([(max_t_b + i * T), 0, prev_ang_vel[0], prev_ang_pos[0]])
        joint_values['q1'].append([(max_t_b + i * T), 0, prev_ang_vel[1], prev_ang_pos[1]])
        joint_values['q2'].append([(max_t_b + i * T), 0, prev_ang_vel[2], prev_ang_pos[2]])

    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] - joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] - joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] - joints_params[2]['e'] * T
        joint_values['q0'].append(
            [max_plato + max_t_b + i * T, -joints_params[0]['e'], prev_ang_vel[0], prev_ang_pos[0]])
        joint_values['q1'].append(
            [max_plato + max_t_b + i * T, -joints_params[1]['e'], prev_ang_vel[1], prev_ang_pos[1]])
        joint_values['q2'].append(
            [max_plato + max_t_b + i * T, -joints_params[2]['e'], prev_ang_vel[2], prev_ang_pos[2]])

    return joint_values


def lin():
    pass


def arc():
    pass

def draw(values):
    inital_values_q0 = values['q0'][:]
    inital_values_q1 = values['q1'][:]
    inital_values_q2 = values['q2'][:]
    values = [[[],[],[]],[[],[],[]],[[],[],[]]]
    time = []

    for value in inital_values_q0:
        time.append(value[0])
        values[0][0].append(value[1])
        values[0][1].append(value[2])
        values[0][2].append(value[3])

    for value in inital_values_q1:
        values[1][0].append(value[1])
        values[1][1].append(value[2])
        values[1][2].append(value[3])

    for value in inital_values_q2:
        values[2][0].append(value[1])
        values[2][1].append(value[2])
        values[2][2].append(value[3])

    plt.plot( time, values[0][0], 'r')
    plt.plot( time, values[1][0], 'b')
    plt.plot( time, values[2][0], 'g')
    plt.legend(['q0', 'q1', 'q2'], loc='upper left')
    plt.title('Joint accelerations. Joint space.')
    plt.xlabel('t')
    plt.ylabel('a')
    plt.show()

    plt.plot( time, values[0][1], 'r')
    plt.plot( time, values[1][1], 'b')
    plt.plot( time, values[2][1], 'g')
    plt.legend(['q0', 'q1', 'q2'], loc='upper left')
    plt.title('Joint velocity. Joint space.')
    plt.xlabel('t')
    plt.ylabel('v')
    plt.show()

    plt.plot( time, values[0][2], 'r')
    plt.plot( time, values[1][2], 'b')
    plt.plot( time, values[2][2], 'g')
    plt.legend(['q0', 'q1', 'q2'], loc='upper left')
    plt.title('Joint position. Joint space.')
    plt.xlabel('t')
    plt.ylabel('x')
    plt.show()

if __name__ == '__main__':
    # print(fk([0, 0, 0]))
    # print(fk([2.0, -2.6, 0.1]))
    # print(ik(fk([0.7, 0, 0.1])))
    # print(ik(fk([0.7, 0.4, 0.1])))
    # jacobian([0.7, 0, 0.1])
    joint_values = ptp([0.37, 0.31, 0.3], [1.1, 1.38, 0.4])
    draw(joint_values)
    joint_values = ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
    draw(joint_values)
    # ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
