# save_as_yaml_from_npy.py
import numpy as np
import yaml
import sys
from pathlib import Path

# Filenames
K_file = Path("/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/camera_matrix.npy")
D_file = Path("/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/dist_coeffs.npy")
out_yaml = Path("/home/scanhub/dev/Drone-Construction-Mapping/src/drone_middleware/config/camera_from_npy.yaml")

# Load
K = np.load(K_file)
D = np.load(D_file)

# Basic checks
if K.shape != (3,3):
    print(f"Warning: camera_matrix has shape {K.shape} (expected (3,3)). Continuing anyway.")
# Normalize D shape to 1D
D = D.reshape(-1)

print("camera_matrix shape:", K.shape)
print("dist_coeffs shape:", D.shape)

# Prepare YAML-friendly formats
# Flatten K row-major to a single list like your r3live YAML
K_list = [float(x) for x in K.reshape(-1).tolist()]

# Try to ensure we provide the standard 5-coeff OpenCV order:
# [k1, k2, p1, p2, k3]
if D.size >= 5:
    D5 = [float(D[0]), float(D[1]), float(D[2]), float(D[3]), float(D[4])]
else:
    # If you have fewer than 5, pad with zeros; if more, keep the first 5
    padded = list(D.tolist()) + [0.0] * max(0, 5 - D.size)
    D5 = [float(x) for x in padded[:5]]

# Build YAML content (adapt fields to match r3live format)
yaml_dict = {
    "r3live_vio": {
        "image_width": 1920,   # <-- adjust to your actual image size if different
        "image_height": 1080,  # <-- adjust accordingly
        "camera_intrinsic": K_list,
        "camera_dist_coeffs": D5
    }
}

# Print to stdout (human readable)
print("\n--- YAML snippet ---")
print(yaml.dump(yaml_dict, default_flow_style=False, sort_keys=False))

# Optionally write to file
with out_yaml.open("w") as f:
    yaml.dump(yaml_dict, f, default_flow_style=False, sort_keys=False)
print(f"\nWrote: {out_yaml.resolve()}")
