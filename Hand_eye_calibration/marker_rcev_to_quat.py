import numpy as np
from scipy.spatial.transform import Rotation as R

rvecs_camera = {
    4: np.array([-2.861, 0.133, -0.841]),
    0: np.array([-3.334, -0.162, -0.815]),
    1: np.array([0.233, -2.571, 1.026]),
    3: np.array([2.363, 0.073, -0.387])
}

tvecs_camera = {
    4: np.array([0.585, 0.165, 0.741]),
    0: np.array([0.780, -0.139, 1.011]),
    1: np.array([0.110, -0.097, 0.549]),
    3: np.array([0.046, 0.135, 0.431])
}

for marker_id in [4,0,1,3]:
    rot = R.from_rotvec(rvecs_camera[marker_id])
    quat_xyzw = rot.as_quat()
    print(f"ID {marker_id}  quat_xyzw (camera): {quat_xyzw}")



