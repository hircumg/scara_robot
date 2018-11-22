#!/usr/bin/env python


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

def PTP():
    pass

def LIN():
    pass

def ARC():
    pass

