"""
单纯的几何计算，负责计算路径与矩形是怎么相交的，被hole_detector.py调用

"""

import math


_EPS = 1e-12
_PROJECTION_WINDOW = 10
_PROJECTION_FALLBACK_DISTANCE = 1.0

# 判断机器人是否在规定的矩形内
def point_in_rect(x, y, x_min, x_max, y_min, y_max):
    return x_min <= x <= x_max and y_min <= y <= y_max

# 
def segment_rect_interval(p0, p1, rect):
    x_min, x_max, y_min, y_max = rect
    x0, y0 = p0
    x1, y1 = p1
    dx = x1 - x0
    dy = y1 - y0

    # 通过进洞和出洞的进度来就算出入口的坐标

    t0 = 0.0   # 进入隧洞时占全局规划的百分比
    t1 = 1.0   # 离开隧洞时占全局规划的百分比

    for p, q in (
        #以下是矩形的四条线
        (-dx, x0 - x_min),
        (dx, x_max - x0),
        (-dy, y0 - y_min),
        (dy, y_max - y0),
    ):
        if abs(p) <= _EPS:
            if q < 0.0:
                return None
            continue

        r = q / p
        if p < 0.0:
            if r > t1:
                return None
            if r > t0:
                t0 = r
        else:
            if r < t0:
                return None
            if r < t1:
                t1 = r

    if t0 > t1:
        return None

    return t0, t1


def segment_rect_entry(p0, p1, rect):
    interval = segment_rect_interval(p0, p1, rect)
    if interval is None:
        return None
    return interval[0]


def cumulative_lengths(points):
    if not points:
        return []

    lengths = [0.0]
    for i in range(1, len(points)):
        x0, y0 = points[i - 1]
        x1, y1 = points[i]
        lengths.append(lengths[-1] + math.hypot(x1 - x0, y1 - y0))
    return lengths


def _search_projection(points, cumulative, robot_xy, start_index, end_index):
    robot_x, robot_y = robot_xy
    best_distance_sq = math.inf
    best_index = start_index
    best_t = 0.0

    for i in range(start_index, end_index + 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        dx = x1 - x0
        dy = y1 - y0
        segment_sq = dx * dx + dy * dy

        if segment_sq <= _EPS:
            t = 0.0
        else:
            t = ((robot_x - x0) * dx + (robot_y - y0) * dy) / segment_sq
            t = max(0.0, min(1.0, t))

        closest_x = x0 + t * dx
        closest_y = y0 + t * dy
        distance_sq = (
            (robot_x - closest_x) * (robot_x - closest_x)
            + (robot_y - closest_y) * (robot_y - closest_y)
        )

        if distance_sq < best_distance_sq:
            best_distance_sq = distance_sq
            best_index = i
            best_t = t

    segment_length = cumulative[best_index + 1] - cumulative[best_index]
    progress = cumulative[best_index] + best_t * segment_length
    return best_distance_sq, progress, best_index, best_t


def project_robot_on_path(
    points,
    cumulative,
    robot_xy,
    hint_index=None,
    local_window=_PROJECTION_WINDOW,
):
    if not points or not cumulative:
        return None

    if len(points) == 1:
        return 0.0, 0, 0.0

    last_index = len(points) - 2

    if hint_index is None:
        start_index = 0
        end_index = last_index
        local_search = False
    else:
        start_index = max(0, hint_index - local_window)
        end_index = min(last_index, hint_index + local_window)
        local_search = True

    best = _search_projection(
        points,
        cumulative,
        robot_xy,
        start_index,
        end_index,
    )

    if local_search and best[0] > _PROJECTION_FALLBACK_DISTANCE ** 2:
        full = _search_projection(
            points,
            cumulative,
            robot_xy,
            0,
            last_index,
        )
        if full[0] < best[0]:
            best = full

    return best[1], best[2], best[3]


def first_entry_arc_length(points, cumulative, rect):
    if not points or not cumulative:
        return None

    x_min, x_max, y_min, y_max = rect

    if len(points) == 1:
        x, y = points[0]
        if point_in_rect(x, y, x_min, x_max, y_min, y_max):
            return 0.0
        return None

    for i in range(len(points) - 1):
        interval = segment_rect_interval(points[i], points[i + 1], rect)
        if interval is None:
            continue

        t_enter = interval[0]
        segment_length = cumulative[i + 1] - cumulative[i]
        return cumulative[i] + t_enter * segment_length

    return None
