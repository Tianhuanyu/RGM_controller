import copy
import math
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy
from tcp import generate_control_command

max_velocity = 2.0 # rad/s (150deg/s ~ 2.5rad/s)
lines_per_round = 524288.0

position_calibrations = [lines_per_round, lines_per_round-43690, -142000+lines_per_round, 217700, lines_per_round-36410, 0] # to_driver_count = phy_count + cali
# [,, -up, +up, -up, 0]

def driver_unit_convert(type, no, value, to_driver = True, relative = False):
    if type == 'p':
        if relative:
            if to_driver:
                return int((value/(2*math.pi)*lines_per_round+lines_per_round)%(2*lines_per_round)-lines_per_round)
            else:
                raise Exception("Not supporting relative to_physics")
                return 0
        if to_driver:
            return int((value/(2*math.pi)*lines_per_round+position_calibrations[no])%lines_per_round)
        else:
            return ((value-position_calibrations[no])/lines_per_round*(2*math.pi)+math.pi)%(2*math.pi)-math.pi
    if type == 'v':
        # TODO
        if to_driver:
            return int(value)
        else:
            return value
    if type == 't':
        # torque
        if to_driver:
            return int(value)
        else:
            return value

def get_trajectory_step_command(points, i):
    command = [None, 1]
    orig_i_last = i-1
    if i:
        while(i < len(points)):
            traj_last_pos = points[orig_i_last].positions
            traj_rel_ns = points[i].time_from_start.to_nsec() - points[orig_i_last].time_from_start.to_nsec()

            if traj_rel_ns < 1000000:
                if i == len(points)-1:
                    # Send the last point out regardless of interval
                    traj_rel_ns = 1000000
                else:
                    i += 1
                    continue

            command[0] = '{0};T[{1:x}]'.format(
                # 6bit relative
                generate_control_command('P', [driver_unit_convert('p', pi, tp-traj_last_pos[pi], True, True) for pi, tp in enumerate(points[i].positions)]),
                # generate_control_command('V', [driver_unit_convert('v', i, p) for p in points[i].velocities]),
                traj_rel_ns/1000000
            )
            break
    command[1] = i+1
    return command

# Returns (q, qdot, qddot) for sampling the JointTrajectory at time t.
# The time t is the time since the trajectory was started.
def sample_traj(traj, t):
    # First point
    if t <= 0.0:
        return copy.deepcopy(traj.points[0])
    # Last point
    if t >= traj.points[-1].time_from_start.to_sec():
        return copy.deepcopy(traj.points[-1])

    # Finds the (middle) segment containing t
    i = 0
    while traj.points[i+1].time_from_start.to_sec() < t:
        i += 1
    return interp_cubic(traj.points[i], traj.points[i+1], t)

def traj_is_finite(traj):
    for pt in traj.points:
        for p in pt.positions:
            if math.isinf(p) or math.isnan(p):
                return False
        for v in pt.velocities:
            if math.isinf(v) or math.isnan(v):
                return False
    return True

def traj_is_finite(traj):
    for pt in traj.points:
        for p in pt.positions:
            if math.isinf(p) or math.isnan(p):
                return False
        for v in pt.velocities:
            if math.isinf(v) or math.isnan(v):
                return False
    return True

def has_limited_velocities(traj):
    for p in traj.points:
        for v in p.velocities:
            if math.fabs(v) > max_velocity:
                return False
    return True

def has_enough_velocities(traj):
    for p in traj.points:
        if len(p.velocities) != len(p.positions):
            return False
    return True

def within_tolerance(a_vec, b_vec, tol_vec):
    for a, b, tol in zip(a_vec, b_vec, tol_vec):
        if abs(a - b) > tol:
            return False
    return True

def reorder_traj_joints(traj, joint_names):
    order = list()
    for j in joint_names:
        try:
            order.append(traj.joint_names.index(j))
        except ValueError:
            order.append(-1)

    new_points = list()
    for p in traj.points:
        new_points.append(JointTrajectoryPoint(
            positions = [(p.positions[i] if i != -1 else 0) for i in order],
            velocities = [(p.velocities[i] if i != -1 else 0) for i in order] if p.velocities else [],
            accelerations = [(p.accelerations[i] if i != -1 else 0) for i in order] if p.accelerations else [],
            time_from_start = p.time_from_start))
    traj.joint_names = joint_names
    traj.points = new_points