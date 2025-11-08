#!/usr/bin/env python
import numpy as np
import yaml
import importlib.util
from pathlib import Path

# --- EDIT THESE ---
IMAGE_WIDTH  = 1920
IMAGE_HEIGHT = 1080
CAMERA_NAME  = "rear_camera"

# Paths to your files
K_PATH  = "/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/camera_matrix.npy"   # 3x3 intrinsic matrix
D_PATH  = "/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/dist_coeffs.npy"     # preferred: a .npy vector like [k1,k2,p1,p2,k3] or fisheye [k1,k2,k3,k4]
# If you only have dist_coeffs.py defining a variable (e.g., dist_coeffs = [...]),
# set D_PY_PATH and leave D_PATH as None:
D_PY_PATH = None  # e.g., "dist_coeffs.py"

def load_D_from_py(py_path, var_name_candidates=("dist_coeffs","D","distCoeffs")):
    spec = importlib.util.spec_from_file_location("dist_module", py_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    for name in var_name_candidates:
        if hasattr(mod, name):
            return np.asarray(getattr(mod, name)).astype(float).ravel()
    raise ValueError(f"No coefficients found in {py_path}. Tried names: {var_name_candidates}")

# Load K
K = np.load(K_PATH).astype(float)
if K.shape != (3,3):
    raise ValueError(f"Expected K to be 3x3, got {K.shape}")

# Load D (prefer .npy; fall back to .py)
if D_PATH and Path(D_PATH).exists():
    D = np.load(D_PATH).astype(float).ravel()
elif D_PY_PATH:
    D = load_D_from_py(D_PY_PATH)
else:
    raise FileNotFoundError("Provide dist_coeffs via dist_coeffs.npy or dist_coeffs.py")

# Decide distortion model
# OpenCV pinhole (rad-tan) usually len D = 5 (k1,k2,p1,p2,k3) or 8/12 with higher-order terms.
# OpenCV fisheye model usually len D = 4 (k1,k2,k3,k4).
if len(D) == 4:
    distortion_model = "fisheye"
elif len(D) in (5,8,12):
    distortion_model = "plumb_bob"
else:
    # default to plumb_bob but warn
    print(f"Warning: unusual D length {len(D)}. Defaulting to plumb_bob.")
    distortion_model = "plumb_bob"

# Rectification (R) and projection (P) for a single camera (not rectified/stereo):
R = np.eye(3)
# P = [ [fx, 0, cx, 0],
#       [0, fy, cy, 0],
#       [0,  0,  1, 0] ]
fx, fy = K[0,0], K[1,1]
cx, cy = K[0,2], K[1,2]
P = np.array([[fx, 0,  cx, 0],
              [0,  fy, cy, 0],
              [0,   0,  1, 0]], dtype=float)

# Build YAML dict in ROS camera_info format
ci = {
    "image_width":  IMAGE_WIDTH,
    "image_height": IMAGE_HEIGHT,
    "camera_name":  CAMERA_NAME,
    "camera_matrix": {
        "rows": 3, "cols": 3, "data": K.reshape(-1).tolist()
    },
    "distortion_model": distortion_model,
    "distortion_coefficients": {
        "rows": 1, "cols": len(D), "data": D.tolist()
    },
    "rectification_matrix": {
        "rows": 3, "cols": 3, "data": R.reshape(-1).tolist()
    },
    "projection_matrix": {
        "rows": 3, "cols": 4, "data": P.reshape(-1).tolist()
    }
}

out_path = Path(f"/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/{CAMERA_NAME}.yaml")
with open(out_path, "w") as f:
    yaml.safe_dump(ci, f, sort_keys=False)

print(f"Wrote {out_path.resolve()}")
