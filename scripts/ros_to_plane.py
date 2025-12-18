import math, json, ast

try:
    import Rhino.Geometry as rg
except ImportError:
    rg = None


def _rotation_matrix_from_rpy(roll_rad, pitch_rad, yaw_rad):
    """從 RPY 歐拉角計算旋轉矩陣 (ZYX 慣例)
    
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    
    Args:
        roll_rad: Roll 角度 (弧度)
        pitch_rad: Pitch 角度 (弧度)
        yaw_rad: Yaw 角度 (弧度)
    
    Returns:
        3x3 旋轉矩陣 (list of lists)
    """
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    cy = math.cos(yaw_rad)
    sy = math.sin(yaw_rad)
    
    # R = Rz * Ry * Rx
    # 第一列
    r11 = cy * cp
    r12 = cy * sp * sr - sy * cr
    r13 = cy * sp * cr + sy * sr
    
    # 第二列
    r21 = sy * cp
    r22 = sy * sp * sr + cy * cr
    r23 = sy * sp * cr - cy * sr
    
    # 第三列
    r31 = -sp
    r32 = cp * sr
    r33 = cp * cr
    
    return [
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ]


def ros_to_plane(x_m, y_m, z_m, roll_val, pitch_val, yaw_val, use_radians=False):
    """將 ROS 座標轉換為 Rhino Plane（只回傳 Plane）

    Args:
        x_m: X 座標 (米)
        y_m: Y 座標 (米)
        z_m: Z 座標 (米)
        roll_val: Roll 角度 (度或弧度，取決於 use_radians)
        pitch_val: Pitch 角度 (度或弧度，取決於 use_radians)
        yaw_val: Yaw 角度 (度或弧度，取決於 use_radians)
        use_radians: 若為 True，輸入為弧度；若為 False，輸入為度數 (預設)

    Returns:
        Rhino.Geometry.Plane 物件
    """
    if rg is None:
        raise RuntimeError("Rhino.Geometry not available. This script is intended for Rhino/Grasshopper.")

    # 位置轉換：米 -> 毫米
    x_mm = x_m * 1000.0
    y_mm = y_m * 1000.0
    z_mm = z_m * 1000.0
    origin = rg.Point3d(x_mm, y_mm, z_mm)

    # 角度轉換：根據 use_radians 決定是否需要轉換
    if use_radians:
        roll_rad = float(roll_val)
        pitch_rad = float(pitch_val)
        yaw_rad = float(yaw_val)
    else:
        roll_rad = math.radians(roll_val)
        pitch_rad = math.radians(pitch_val)
        yaw_rad = math.radians(yaw_val)

    # 計算旋轉矩陣
    R = _rotation_matrix_from_rpy(roll_rad, pitch_rad, yaw_rad)

    # 從旋轉矩陣提取軸向量（列向量）
    x_axis = rg.Vector3d(R[0][0], R[1][0], R[2][0])
    y_axis = rg.Vector3d(R[0][1], R[1][1], R[2][1])
    # z 軸可由 x,y 推得，亦可直接使用矩陣第三列
    # 這裡仍計算並單位化，雖然不回傳
    z_axis = rg.Vector3d(R[0][2], R[1][2], R[2][2])
    x_axis.Unitize(); y_axis.Unitize(); z_axis.Unitize()

    # 建立 Plane 並只回傳 plane
    plane = rg.Plane(origin, x_axis, y_axis)
    return plane


# Grasshopper interface: three inputs
# - pose: list/tuple [x, y, z, roll, pitch, yaw] (meters, degrees/radians)
# - ros_pose: string like "[-0.5, 1.0, 0.05, 0.0, 0.0, 90.0]" (Panel), or list; takes priority over pose when both provided
# - use_radians: bool toggle (True = radians, False = degrees, default: False)

def _is_str(v):
    try:
        basestring  # type: ignore[name-defined]
        return isinstance(v, basestring)  # type: ignore[name-defined]
    except NameError:
        return isinstance(v, str)


def _coerce_list6(v):
    if isinstance(v, (list, tuple)) and len(v) == 6:
        try:
            return [float(x) for x in v]
        except Exception:
            return None
    return None


def _coerce_any6(v):
    # Accept list/tuple directly, or parse string "[x,y,z,roll,pitch,yaw]"
    vals = _coerce_list6(v)
    if vals is not None:
        return vals
    if _is_str(v):
        try:
            try:
                arr = json.loads(v)
            except Exception:
                arr = ast.literal_eval(v)
            return _coerce_list6(arr)
        except Exception:
            return None
    return None

pose_vals = None
ros_vals = None
radians_mode = False

try:
    use_radians  # type: ignore[name-defined]
    radians_mode = bool(use_radians)  # type: ignore[name-defined]
except NameError:
    pass

try:
    ros_pose  # type: ignore[name-defined]
    ros_vals = _coerce_any6(ros_pose)  # type: ignore[name-defined]
except NameError:
    pass

try:
    pose  # type: ignore[name-defined]
    pose_vals = _coerce_list6(pose)  # type: ignore[name-defined]
except NameError:
    pass

vals = ros_vals if ros_vals is not None else pose_vals
if rg is not None and vals:
    x_m, y_m, z_m, roll_val, pitch_val, yaw_val = vals
    plane = ros_to_plane(x_m, y_m, z_m, roll_val, pitch_val, yaw_val, radians_mode)
