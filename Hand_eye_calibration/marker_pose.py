import numpy as np
from scipy.spatial.transform import Rotation as R

# Recorded quaternion when touching 
tool_quat_xyzw = np.array([0.0, 0.0, 0.0, 0.01])

# Create rotation object from recorded quat
tool_rot = R.from_quat(tool_quat_xyzw)

# 180 deg around X axis 
flip_rot = R.from_euler('x', 180, degrees=True)

# Compose: marker orientation = tool orientation * flip
marker_rot = tool_rot * flip_rot

# Get new quaternion (xyzw order)
marker_quat_xyzw = marker_rot.as_quat()

# Set print options to avoid scientific notation
np.set_printoptions(suppress=True)

print("Marker quaternion (xyzw):", np.round(marker_quat_xyzw, 4))