#!/usr/bin/env python
import numpy as np
from math import atan, sqrt, sin, acos, cos, fabs, atan2
from numpy import sign
from matplotlib import pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D

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
    p0 = T[:3, 3]

    p = [0,0,0]
    p[0] = links[1]*cos(q[0]) + links[2]*cos(q[0]+q[1])
    p[1] = links[1]*sin(q[0]) + links[2]*sin(q[0]+q[1])
    p[2] = links[0] + q[2] - links[3]
    # print(p0, p)
    # p = [round(i,2) for i in p]
    return p


def ik(p):
    global model
    links = model['links']
    q = [0, 0, 0]

    d = sqrt(p[0] ** 2 + p[1] ** 2)
    val = (-links[2] ** 2 + links[1] ** 2 + p[0] ** 2 + p[1] ** 2) / (2 * links[1] * d)
    val = val if abs(val) < 1.000 else 1.000 * sign(val)
    # phi1 = acos(val)
    phi1 = atan2(sqrt(1-val**2),val)
    phi2 = atan2(p[1], p[0]) if p[0] != 0 else 0

    q[0] = (-phi1 + phi2)
    val = (p[0] ** 2 + p[1] ** 2 - links[1] ** 2 - links[2] ** 2) / (2 * links[1] * links[2])
    # print("q_2 val: ", val)
    val = val if abs(val) < 1.000 else 1.000 * sign(val)
    # q[1] = acos(val)
    q[1] = atan2(sqrt(1-val**2),val)
    q[2] = p[2] + links[3] - links[0]
    print("d:", d, ", phi_1: ", phi1, ", phi_2: ", phi2, ", q_1:", q[0], ", q_2: ", q[1])
    # q = np.round(q, 6)
    # q = [round(i,2) for i in q]
    return q

def ik2(p):
    global model
    links = model['links']
    q = [0, 0, 0]

    d = sqrt(p[0] ** 2 + p[1] ** 2)


    val1 = (p[0] ** 2 + p[1] ** 2 - links[1] ** 2 - links[2] ** 2) / (2 * links[1] * links[2])
    # print("q_2 val: ", val)
    val1 = val1 if abs(val1) < 1.000 else 1.000 * sign(val1)
    # q[1] = acos(val)
    q[1] = atan2(sqrt(1-val1**2),val1)

    # val = (-links[2] ** 2 + links[1] ** 2 + p[0] ** 2 + p[1] ** 2) / (2 * links[1] * d)
    # val = val if abs(val) < 1.000 else 1.000 * sign(val)
    # phi1 = acos(val)

    phi1 = -atan2(links[2]*cos(q[1]), links[1] + links[2]*sin(q[1]))
    phi2 = atan2(p[1], p[0])
    q[0] = (phi1 + phi2)

    q[2] = p[2] + links[3] - links[0]
    # print("d:", d, ", phi_1: ", phi1, ", phi_2: ", phi2, ", q_1:", q[0], ", q_2: ", q[1])
    # q = np.round(q, 6)
    # q = [round(i,2) for i in q]
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
    joints_params.append({'e': ((q_f[0] - q_0[0]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((q_f[0] - q_0[0]) / (max_plato + max_t_b))})
    joints_params.append({'e': ((q_f[1] - q_0[1]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((q_f[1] - q_0[1]) / (max_plato + max_t_b))})
    joints_params.append({'e': ((q_f[2] - q_0[2]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((q_f[2] - q_0[2]) / (max_plato + max_t_b))})
    print(joints_params)

    joint_values = []
    prev_ang_vel = [0, 0, 0]
    prev_ang_pos = q_0
    joint_values.append(prev_ang_pos.copy())

    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] + joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] + joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] + joints_params[2]['e'] * T
        joint_values.append(prev_ang_pos.copy())

    print("Increasing: ", len(joint_values))

    for i in range(int(max_plato / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + joints_params[0]['w'] * T
        prev_ang_pos[1] = prev_ang_pos[1] + joints_params[1]['w'] * T
        prev_ang_pos[2] = prev_ang_pos[2] + joints_params[2]['w'] * T
        joint_values.append(prev_ang_pos.copy())

    print("Plato: ", len(joint_values))

    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] - joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] - joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] - joints_params[2]['e'] * T
        joint_values.append(prev_ang_pos.copy())

    print("Decreasing: ", len(joint_values))

    return joint_values


def integrate(values):
    integrate_values = []
    prev_values = [0, 0, 0]
    for value in values:
        prev_values[0] = prev_values[0] + value[0]
        prev_values[1] = prev_values[1] + value[1]
        prev_values[2] = prev_values[2] + value[2]
        integrate_values.append(prev_values.copy())
    return integrate_values


def diffirentiate(values):
    diff_values = []
    prev_values = [0, 0, 0]
    d = [0, 0, 0]
    for value in values:
        d[0] = value[0] - prev_values[0]
        d[1] = value[1] - prev_values[1]
        d[2] = value[2] - prev_values[2]
        diff_values.append(d.copy())
        prev_values = value.copy()

    diff_values.pop(0)
    return diff_values


def lin(p_0, p_f, j=0):
    global v_max, f, a_max
    T = 1 / f
    print(p_0, p_f)
    times = []
    times.append({'b': (v_max / a_max), 'p': (abs(p_f[0] - p_0[0]) / v_max - (v_max / a_max)),
                  'b_triangle': (sqrt(abs(p_f[0] - p_0[0]) / a_max))})
    times.append({'b': (v_max / a_max), 'p': (abs(p_f[1] - p_0[1]) / v_max - (v_max / a_max)),
                  'b_triangle': (sqrt(abs(p_f[1] - p_0[1]) / a_max))})
    times.append({'b': (v_max / a_max), 'p': (abs(p_f[2] - p_0[2]) / v_max - (v_max / a_max)),
                  'b_triangle': (sqrt(abs(p_f[2] - p_0[2]) / a_max))})
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
    joints_params.append({'e': ((p_f[0] - p_0[0]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((p_f[0] - p_0[0]) / (max_plato + max_t_b))})
    joints_params.append({'e': ((p_f[1] - p_0[1]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((p_f[1] - p_0[1]) / (max_plato + max_t_b))})
    joints_params.append({'e': ((p_f[2] - p_0[2]) / ((max_plato + max_t_b) * max_t_b)),
                          'w': ((p_f[2] - p_0[2]) / (max_plato + max_t_b))})
    print(joints_params)



    joint_values = []
    velosity_values = []
    prev_ang_vel = [0, 0, 0]
    prev_ang_pos = p_0
    # velosity_values.append(prev_ang_vel.copy())
    # joint_values.append(prev_ang_pos.copy())
    joint_values.append(ik(prev_ang_pos.copy()))
    # joint_values.append(fk(ik(prev_ang_pos.copy())))
    debug = []
    debug.append([prev_ang_pos.copy(), fk(ik(prev_ang_pos.copy()))])
    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] + joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] + joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] + joints_params[2]['e'] * T
        # prev_ang_pos = [round(i,2) for i in prev_ang_pos]
        debug.append([prev_ang_pos.copy(), fk(ik(prev_ang_pos.copy()))])
        # velosity_values.append(prev_ang_vel.copy())
        # joint_values.append(prev_ang_pos.copy())
        joint_values.append(ik(prev_ang_pos.copy()))
        # joint_values.append(fk(ik(prev_ang_pos.copy())))

    print("Increasing: ", len(joint_values))

    for i in range(int(max_plato / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + joints_params[0]['w'] * T
        prev_ang_pos[1] = prev_ang_pos[1] + joints_params[1]['w'] * T
        prev_ang_pos[2] = prev_ang_pos[2] + joints_params[2]['w'] * T
        # prev_ang_pos = [round(i,2) for i in prev_ang_pos]
        debug.append([prev_ang_pos.copy(), fk(ik(prev_ang_pos.copy()))])
        # velosity_values.append(prev_ang_vel.copy())
        # joint_values.append(prev_ang_pos.copy())
        joint_values.append(ik(prev_ang_pos.copy()))
        # joint_values.append(fk(ik(prev_ang_pos.copy())))


    print("Plato: ", len(joint_values))

    for i in range(int(max_t_b / T)):
        prev_ang_pos[0] = prev_ang_pos[0] + prev_ang_vel[0] * T
        prev_ang_pos[1] = prev_ang_pos[1] + prev_ang_vel[1] * T
        prev_ang_pos[2] = prev_ang_pos[2] + prev_ang_vel[2] * T
        prev_ang_vel[0] = prev_ang_vel[0] - joints_params[0]['e'] * T
        prev_ang_vel[1] = prev_ang_vel[1] - joints_params[1]['e'] * T
        prev_ang_vel[2] = prev_ang_vel[2] - joints_params[2]['e'] * T
        # prev_ang_pos = [round(i,2) for i in prev_ang_pos]
        debug.append([prev_ang_pos.copy(), fk(ik(prev_ang_pos.copy()))])
        # velosity_values.append(prev_ang_vel.copy())
        # joint_values.append(prev_ang_pos.copy())
        joint_values.append(ik(prev_ang_pos.copy()))
        # joint_values.append(fk(ik(prev_ang_pos.copy())))


    print("Decreasing: ", len(joint_values))

    return joint_values, debug
    # return velosity_values


def arc():
    pass




def convert_to_cartesian(values):
    cartesian = []
    for value in values:
        cartesian.append(fk(value))
    return cartesian



def draw(values):
    global f
    T = 1 / f
    q_0 = [v[0] for v in values]
    q_1 = [v[1] for v in values]
    q_2 = [v[2] for v in values]
    time = [i * T for i in range(len(values))]

    plt.plot(time, q_0, 'r')
    plt.plot(time, q_1, 'b')
    plt.plot(time, q_2, 'g')
    plt.legend(['q0', 'q1', 'q2'], loc='upper left')
    plt.title('Joint velocity. Joint space.')
    plt.xlabel('t')
    plt.ylabel('v')
    plt.show()


def draw3d(values):
    global f
    T = 1 / f
    q_0 = [v[0] for v in values]
    q_1 = [v[1] for v in values]
    q_2 = [v[2] for v in values]
    time = [i * T for i in range(len(values))]

    mpl.rcParams['legend.fontsize'] = 10

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.plot(q_0, q_1, q_2, label='velosity')
    ax.legend()

    plt.show()


if __name__ == '__main__':
    q0 = [0,0.1,0]

    print("fk: ", fk(q0))
    print("q0: ", q0)

    print("ik lec: ", ik(fk(q0)))
    print("ik my: ", ik2(fk(q0)))
    # print(ik([1.80, 0.0003, 0.3]))
    # print(ik([1.7998, 0.0003, 0.3]))
    # print(ik([1.7992, 0.0471, 0.3]))
    # print(fk(ik([1.80, 0.0003, 0.3])))
    # print(fk(ik([1.7998, 0.0003, 0.3])))
    # print(fk(ik([1.7992, 0.0471, 0.3])))
    # print(ik([0.7, 0.43, 0.3]))     # [-0.54, 2.19, 0.0]
    # print(fk([-0.54, 2.19, 0.0]))
    # print(fk(ik([1.8, 0, 0.3])))
    # print(fk(ik([0.7,  0.43, 0.3])))
    # print(fk(ik([0.7, 0.4, 0.1])))
    # print(jacobian([0.7, 0, 0.1])[0:3])
    # exit(0)
    joint_values = ptp([0.37, 0.31, 0.3], [1.1, 1.38, 0.4])
    # joint_values = ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
    # joint_values, debug = lin([1.8, 0, 0.3], [1.1, 1.38, 0.3])
    # draw(joint_values)

    # draw(convert_to_cartesian(joint_values))
    # draw(integrate(joint_values))
    draw(diffirentiate(joint_values))
    # draw(diffirentiate(convert_to_cartesian(joint_values)))
    # draw3d(joint_values)
    # draw3d(convert_to_cartesian(joint_values))
    # draw3d(integrate(joint_values))
    # draw3d(diffirentiate(joint_values))
    # convert_to_cartesian(joint_values)
    # joint_values = [[joint_values['q0'][i][3],joint_values['q1'][i][3],joint_values['q2'][i][3]] for i in range(len(joint_values['q0']))]
    # draw_cartesian(joint_values)
    # joint_values = ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
    # draw(joint_values)
    # ptp([1.8, 0, 0.3], [1.1, 1.38, 0.4])
