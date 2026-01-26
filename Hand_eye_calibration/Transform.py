import numpy as np
from scipy.spatial.transform import Rotation as R

# Compute per-marker transforms
base_positions = [np.array([0.2644, 0.4008, 0.3694]),
                  np.array([-0.2657, 0.4083, 0.3808]),
                  np.array([0.2648, 0.6870, 0.6568]),
                  np.array([-0.2513, 0.6860, 0.6547])
                  ]
base_quats = [np.array([-0.09546, 0.36578, -0.8967, -0.2302]),
              np.array([0.1145, 0.3425, -0.8921, 0.2715]),
              np.array([-0.2630, 0.4015, -0.9141 , -0.0498]),
              np.array([0.079, 0.3750, -0.8983, 0.2144])]

cam_positions = [np.array([0.110, -0.097, 0.549]),#1
                np.array([0.780, -0.139, 1.011]),#0
                np.array([0.046, 0.135, 0.431]),#3
                np.array([0.585, 0.165, 0.741])#4
                ]
cam_quats = [np.array([0.0825, -0.9102, 0.3632, 0.1808]),
            np.array([-0.9598, -0.04664, -0.2346, -0.1467]),
            np.array([0.9186, 0.0283, -0.1504, 0.3644]),
            np.array([-0.9555, 0.0444, -0.2808, 0.07821])]
# Compute T_base_camera for each marker observation

T_base_cam_estimates = []
for i in range(4):
    T_base_m = np.eye(4)
    T_base_m[:3, 3] = base_positions[i]
    T_base_m[:3, :3] = R.from_quat(base_quats[i]).as_matrix()

    T_cam_m = np.eye(4)
    T_cam_m[:3, 3] = cam_positions[i]
    T_cam_m[:3, :3] = R.from_quat(cam_quats[i]).as_matrix()

    T_base_cam = T_base_m @ np.linalg.inv(T_cam_m)
    T_base_cam_estimates.append(T_base_cam)

# Average translation
avg_pos = np.mean([T[:3, 3] for T in T_base_cam_estimates], axis=0)

# Average rotation 
avg_rot_mat = np.mean([T[:3, :3] for T in T_base_cam_estimates], axis=0)
U, _, Vt = np.linalg.svd(avg_rot_mat)
avg_rot = U @ Vt

T_base_camera = np.eye(4)
T_base_camera[:3, :3] = avg_rot
T_base_camera[:3, 3] = avg_pos

print("Estimated T_base_camera:\n", T_base_camera)
print("Position (m):", T_base_camera[:3, 3])
rot = R.from_matrix(T_base_camera[:3, :3])
print("Orientation (quat):", rot.as_quat())  # x,y,z,w
