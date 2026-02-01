import numpy as np
from scipy.spatial.transform import Rotation as R

# Compute per-marker transforms
base_positions = [np.array([0.0050, 0.6524, 0.6232])

                  ]
base_quats = [np.array([-0.3842, -0.0215,  0.0338, -0.9224])
    ]

cam_positions = [np.array([0.241, 0.077, 0.487])#1

                ]
cam_quats = [np.array([0.911, 0.083, -0.130, 0.381])]
# Compute T_base_camera for each marker observation

T_base_cam_estimates = []
for i in range(1):
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
print("Orientation (quaternion xyzw):", rot.as_quat())

