import math
import json
from collections import OrderedDict

try:
    import Rhino.Geometry as rg
except Exception:
    rg = None


def _compute_rpy_from_axes(x_axis, y_axis, z_axis):
    """Compute roll, pitch, yaw (deg) from right-handed axes using ZYX convention.

    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Columns of rotation matrix are the axis unit vectors in world coordinates.
    """
    r11 = x_axis.X
    r12 = y_axis.X
    r13 = z_axis.X
    r21 = x_axis.Y
    r22 = y_axis.Y
    r23 = z_axis.Y
    r31 = x_axis.Z
    r32 = y_axis.Z
    r33 = z_axis.Z

    sy = math.sqrt(r11 * r11 + r21 * r21)
    singular = sy < 1e-9

    if not singular:
        yaw = math.degrees(math.atan2(r21, r11))
        pitch = math.degrees(math.atan2(-r31, sy))
        roll = math.degrees(math.atan2(r32, r33))
    else:
        # Gimbal lock
        yaw = math.degrees(math.atan2(-r12, r22))
        pitch = math.degrees(math.atan2(-r31, sy))
        roll = 0.0

    return roll, pitch, yaw


def _build_axes_from_z(z_axis):
    """Given a Z axis (Vector3d), build a stable right-handed frame (X, Y, Z).

    Uses world X as reference; if nearly parallel, switches to world Y.
    X = Z x refX; Y = Z x X.
    """
    z = rg.Vector3d(z_axis)
    if not z.Unitize():
        raise ValueError("Z axis vector cannot be zero-length")

    ref_x = rg.Vector3d(1.0, 0.0, 0.0)
    if abs(rg.Vector3d.Multiply(z, ref_x)) > 0.99:  # nearly parallel
        ref_x = rg.Vector3d(0.0, 1.0, 0.0)

    x = rg.Vector3d.CrossProduct(z, ref_x)
    _ = x.Unitize()
    y = rg.Vector3d.CrossProduct(z, x)
    _ = y.Unitize()

    return x, y, z


def plane_to_movej(plane):
    """Convert a Rhino Plane to moveJ dict with meters and ZYX Euler angles (deg).

    - Position: mm -> m
    - Orientation: use Plane.XAxis/YAxis/ZAxis directly (deconstruct plane), then RPY (deg)
    - motion_type is always "moveJ"
    """
    if rg is None:
        raise RuntimeError("Rhino.Geometry not available. This script is intended for Rhino/Grasshopper.")

    origin = plane.Origin
    x_m = origin.X / 1000.0
    y_m = origin.Y / 1000.0
    z_m = origin.Z / 1000.0

    # Deconstruct Plane: use plane's axes to preserve its rotation
    x_axis = rg.Vector3d(plane.XAxis); x_axis.Unitize()
    y_axis = rg.Vector3d(plane.YAxis); y_axis.Unitize()
    z_axis = rg.Vector3d(plane.ZAxis); z_axis.Unitize()
    roll_deg, pitch_deg, yaw_deg = _compute_rpy_from_axes(x_axis, y_axis, z_axis)

    result = OrderedDict([
        ("motion_type", "moveJ"),
        ("x", round(x_m, 3)),
        ("y", round(y_m, 3)),
        ("z", round(z_m, 3)),
        ("roll", round(roll_deg, 3)),
        ("pitch", round(pitch_deg, 3)),
        ("yaw", round(yaw_deg, 3)),
    ])
    return result


# Grasshopper convenience: if a variable named `plane` exists in the scope,
# compute outputs `result` and `movej_command` for direct wiring.
try:  # noqa: SIM105
    plane  # type: ignore[name-defined]
    if plane is not None and rg is not None:
        result = plane_to_movej(plane)  # type: ignore[name-defined]
        movej_command = json.dumps(result, sort_keys=False)
except NameError:
    pass
