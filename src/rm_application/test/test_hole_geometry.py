import math

from rm_application.hole_geometry import (
    cumulative_lengths,
    first_entry_arc_length,
    point_in_rect,
    project_robot_on_path,
    segment_rect_entry,
    segment_rect_interval,
)


RECT = (1.0, 2.0, 1.0, 2.0)


def test_point_in_rect():
    assert point_in_rect(1.5, 1.5, *RECT)
    assert point_in_rect(1.0, 1.0, *RECT)
    assert not point_in_rect(0.9, 1.5, *RECT)
    assert not point_in_rect(2.1, 1.5, *RECT)


def test_segment_enters_from_left():
    t = segment_rect_entry((0.0, 1.5), (3.0, 1.5), RECT)
    assert t is not None
    assert math.isclose(t, 1.0 / 3.0)


def test_segment_enters_from_right():
    t = segment_rect_entry((3.0, 1.5), (0.0, 1.5), RECT)
    assert t is not None
    assert math.isclose(t, 1.0 / 3.0)


def test_segment_enters_from_bottom():
    t = segment_rect_entry((1.5, 0.0), (1.5, 3.0), RECT)
    assert t is not None
    assert math.isclose(t, 1.0 / 3.0)


def test_segment_enters_from_top():
    t = segment_rect_entry((1.5, 3.0), (1.5, 0.0), RECT)
    assert t is not None
    assert math.isclose(t, 1.0 / 3.0)


def test_segment_starts_inside():
    interval = segment_rect_interval((1.5, 1.5), (3.0, 1.5), RECT)
    assert interval is not None
    assert math.isclose(interval[0], 0.0)


def test_segment_misses_rect():
    assert segment_rect_entry((0.0, 0.0), (0.5, 0.5), RECT) is None


def test_segment_tangent_to_corner():
    assert segment_rect_entry((0.0, 2.0), (2.0, 0.0), RECT) is not None


def test_cumulative_lengths():
    lengths = cumulative_lengths([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    assert lengths == [0.0, 1.0, 2.0]


def test_project_robot_on_path():
    points = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    cumulative = cumulative_lengths(points)
    progress, index, t = project_robot_on_path(
        points, cumulative, (0.5, 0.2)
    )
    assert index == 0
    assert math.isclose(t, 0.25)
    assert math.isclose(progress, 0.5)


def test_project_robot_on_path_with_hint():
    points = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]
    cumulative = cumulative_lengths(points)
    progress, index, _ = project_robot_on_path(
        points, cumulative, (2.0, 1.0), hint_index=1
    )
    assert index == 1
    assert math.isclose(progress, 3.0)


def test_first_entry_from_left():
    points = [(0.0, 1.5), (3.0, 1.5)]
    cumulative = cumulative_lengths(points)
    entry = first_entry_arc_length(points, cumulative, RECT)
    assert entry is not None
    assert math.isclose(entry, 1.0)


def test_first_entry_from_right():
    points = [(3.0, 1.5), (0.0, 1.5)]
    cumulative = cumulative_lengths(points)
    entry = first_entry_arc_length(points, cumulative, RECT)
    assert entry is not None
    assert math.isclose(entry, 1.0)


def test_first_entry_when_path_starts_inside():
    points = [(1.5, 1.5), (3.0, 1.5)]
    cumulative = cumulative_lengths(points)
    entry = first_entry_arc_length(points, cumulative, RECT)
    assert entry is not None
    assert math.isclose(entry, 0.0)


def test_first_entry_returns_none_when_missing_rect():
    points = [(0.0, 0.0), (0.5, 0.5)]
    cumulative = cumulative_lengths(points)
    assert first_entry_arc_length(points, cumulative, RECT) is None
