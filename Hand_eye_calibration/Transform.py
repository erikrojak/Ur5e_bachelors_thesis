import numpy as np
from scipy.spatial.transform import Rotation as R

# Assume lists: base_positions (4x3), base_quats (4x4), cam_positions (4x3), cam_quats (4x4)
# Compute per-marker transforms
base_positions = [np.array([0.5, 0.0, 0.2]),
                  np.array([0.6, 0.1, 0.25]),
                  np.array([0.4, -0.1, 0.22]),
                  np.array([0.55, 0.05, 0.23])
                  ]
base_quats = [np.array([0.0, 0.0, 0.0, 1.0]),
              np.array([0.0, 0.0, 0.1, 0.995]),
              np.array([0.0, 0.0, -0.1, 0.995]),
              np.array([0.0, 0.0, 0.05, 0.99875])]

cam_positions = [np.array([0.1, 0.0, 0.5]),
                np.array([0.15, 0.05, 0.55]),
                np.array([0.05, -0.05, 0.52]),
                np.array([0.12, 0.02, 0.53])
                ]
cam_quats = [np.array([0.0, 0.0, 0.0, 1.0]),
            np.array([0.0, 0.0, 0.1, 0.995]),
            np.array([0.0, 0.0, -0.1, 0.995]),
            np.array([0.0, 0.0, 0.05, 0.99875])]
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

# Average rotation (simple way)
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
