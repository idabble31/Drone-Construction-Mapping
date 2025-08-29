import math

def Rx(a):
    c,s = math.cos(a), math.sin(a)
    return [[1,0,0],[0,c,-s],[0,s,c]]

def Ry(a):
    c,s = math.cos(a), math.sin(a)
    return [[c,0,s],[0,1,0],[-s,0,c]]

def Rz(a):
    c,s = math.cos(a), math.sin(a)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def mm(A,B):  # 3x3 * 3x3
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def flatten_row_major(R):
    return [R[0][0],R[0][1],R[0][2],
            R[1][0],R[1][1],R[1][2],
            R[2][0],R[2][1],R[2][2]]

# Base: compute IMU(base_link) -> camera_optical from URDF xyz/rpy
def ext_from_urdf(xyz, rpy):
    tx,ty,tz = xyz
    roll,pitch,yaw = rpy
    # URDF fixed-axis RPY: Rz(yaw)*Ry(pitch)*Rx(roll)
    R_base_link = mm(Rz(yaw), mm(Ry(pitch), Rx(roll)))
    # camera_link -> camera_optical (OpenCV): rpy = (-90°, 0, -90°)
    R_link_opt  = mm(Rz(-math.pi/2), Rx(-math.pi/2))
    R_i2c = mm(R_base_link, R_link_opt)
    return flatten_row_major(R_i2c), [tx,ty,tz]

# NEW: same as ext_from_urdf, but with an extra yaw offset about IMU z
# Use yaw_offset = +math.pi for REAR-FACING relative to the same physical mount
def ext_with_yaw_offset(xyz, rpy, yaw_offset_rad):
    tx,ty,tz = xyz
    roll,pitch,yaw = rpy
    R_base_link = mm(Rz(yaw + yaw_offset_rad), mm(Ry(pitch), Rx(roll)))
    R_link_opt  = mm(Rz(-math.pi/2), Rx(-math.pi/2))
    R_i2c = mm(R_base_link, R_link_opt)
    return flatten_row_major(R_i2c), [tx,ty,tz]

# ---------- Examples ----------

# Your "front-right" example from earlier:
R_front, t_front = ext_from_urdf((0.122, 0.0, 0.18), (0.0, 0.0, 1.5708))
print("camera: front (same mount)")
print("camera_ext_R:", ", ".join(f"{v:.8f}" for v in R_front))
print("camera_ext_t:", ", ".join(f"{v:.6f}" for v in t_front))
print()

# Make that SAME physical mount face REAR by adding +pi yaw around IMU z:
R_rear_from_same_mount, t_same = ext_with_yaw_offset((0.122, 0.0, 0.18), (0.0, 0.0, 1.5708), math.pi)
print("camera: rear (same physical mount, +pi yaw offset)")
print("camera_ext_R:", ", ".join(f"{v:.8f}" for v in R_rear_from_same_mount))
print("camera_ext_t:", ", ".join(f"{v:.6f}" for v in t_same))
print()

# If you also MOVE the camera to the back of the drone physically (e.g., x = -0.122)
# keep the same orientation logic (rear-facing = +pi yaw offset), just change xyz:
R_rear_physical, t_rear_physical = ext_with_yaw_offset((0.0, 0.0, 0.095), (1.5708, 0.0, 0.0), math.pi)
print("camera: below (physically at back, +pi yaw offset)")
print("camera_ext_R:", ", ".join(f"{v:.8f}" for v in R_rear_physical))
print("camera_ext_t:", ", ".join(f"{v:.6f}" for v in t_rear_physical))
