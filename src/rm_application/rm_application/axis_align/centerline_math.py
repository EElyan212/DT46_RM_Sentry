import math


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, low, high):
    return max(low, min(value, high))


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def signed_distance_to_line(x, y, point, yaw):
    nx = -math.sin(yaw)
    ny = math.cos(yaw)
    return nx * (x - point[0]) + ny * (y - point[1])


def lateral_velocity(error, kp, deadband, min_speed, max_speed):
    if abs(error) <= deadband:
        return 0.0

    speed = clamp(kp * abs(error), min_speed, max_speed)
    return -math.copysign(speed, error)


def map_velocity_to_base(vx_map, vy_map, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    vx_base = cos_yaw * vx_map + sin_yaw * vy_map
    vy_base = -sin_yaw * vx_map + cos_yaw * vy_map
    return vx_base, vy_base


def slew_limit(current, target, max_delta):
    return current + clamp(target - current, -max_delta, max_delta)


def point_along_axis(point, yaw, distance):
    return (
        point[0] + distance * math.cos(yaw),
        point[1] + distance * math.sin(yaw),
    )
