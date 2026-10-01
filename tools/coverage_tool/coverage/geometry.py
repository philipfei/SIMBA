import math


def wrap(angle):
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
