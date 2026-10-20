from enum import Enum

class TacticalMode(str, Enum):
    """哨兵当前采用的全局战术模式。"""
    DEFENSE = "DEFENSE"
    NEUTRAL = "NEUTRAL"
    PRESSURE = "PRESSURE"

class ControlZoneId(str, Enum):
    """2027 比赛中的三个控制区，只用于描述比赛局势。
    其中只有 CENTER 是哨兵能够直接到达并占领的控制区。
    """
    OWN_HIGH = "OWN_HIGH"
    CENTER = "CENTER"
    ENEMY_HIGH = "ENEMY_HIGH"

class PatrolRegion(str, Enum):
    """哨兵在不同战术模式下允许巡逻的区域。"""
    HOME = "HOME"
    CENTER = "CENTER"
    ENEMY = "ENEMY"

class NavigationTask(str, Enum):
    """导航树同一时刻执行的平移任务。"""
    STOP = "STOP"
    RESUPPLY = "RESUPPLY"
    OCCUPY_CENTER = "OCCUPY_CENTER"
    PATROL = "PATROL"
    CHASE = "CHASE"