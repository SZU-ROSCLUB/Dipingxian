"""Mission goals and competition-course waypoints."""

import numpy as np


TASK1_GOAL = np.array([4.0, 1.75], dtype=float)
TASK3_GOAL = np.array([0.452, 0.355], dtype=float)
STOP_GOAL = np.array([0.25, 0.15], dtype=float)


CLOCKWISE_WAYPOINTS = np.array(
    [
        [2.500, 1.800],
        [2.500, 2.500],
        [2.000, 3.150],
        [1.200, 3.150],
        [0.750, 3.525],
        [0.750, 3.975],
        [1.100, 4.125],
        [3.700, 4.200],
        [4.100, 3.975],
        [4.150, 3.600],
        [3.650, 3.200],
        [3.100, 3.200],
        [2.600, 2.750],
        [2.450, 2.200],
    ],
    dtype=float,
)


COUNTERCLOCKWISE_WAYPOINTS = np.array(
    [
        [2.500, 1.800],
        [2.500, 2.500],
        [3.000, 3.150],
        [3.750, 3.150],
        [4.050, 3.505],
        [4.200, 4.005],
        [3.800, 4.150],
        [1.300, 4.325],
        [0.750, 3.975],
        [0.750, 3.500],
        [1.250, 3.050],
        [1.900, 3.025],
        [2.400, 2.500],
        [2.300, 2.200],
    ],
    dtype=float,
)
