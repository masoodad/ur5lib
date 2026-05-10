import numpy as np

UR5_DH = np.array([
    [ 0.0,      0.1625,   np.pi / 2],   # J1  base rotation
    [-0.425,    0.0,      0.0      ],   # J2  shoulder
    [-0.3922,   0.0,      0.0      ],   # J3  elbow
    [ 0.0,      0.1333,   np.pi / 2],   # J4  wrist 1
    [ 0.0,      0.0997,  -np.pi / 2],   # J5  wrist 2
    [ 0.0,      0.0996,   0.0      ],   # J6  wrist 3
])