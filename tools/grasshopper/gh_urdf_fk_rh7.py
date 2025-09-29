# -*- coding: utf-8 -*-
"""
Grasshopper GhPython script (Rhino 7, IronPython) to load a URDF and perform forward kinematics (no IK).

Inputs (set these as GhPython inputs):
- URDFPath: string, absolute or relative path to .urdf
- J: list of 6 floats, joint_1..joint_6 (if shorter, missing values assumed 0)
- Deg: bool, True if J is in degrees; False for radians
- AxisLen: float, visualization axis length (model units)

Outputs:
- JointNames: list[str]
- LinkNames: list[str]
- JointPlanes: list[Rhino.Geometry.Plane]
- LinkPlanes: list[Rhino.Geometry.Plane]
- EEFPlane: Rhino.Geometry.Plane (tool0 if present, else flange, else last)
- Preview: list[Rhino.Geometry.GeometryBase] (simple axes & spheres)
- Debug: str

Notes:
- Parses <origin xyz rpy>, <axis xyz>, joint types (revolute/fixed/prismatic)
- Rotation composition for origin uses Trans(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll)
- Joint rotation is applied about the joint's local axis (in the joint frame)
- Mesh import is intentionally not included (baking/importing per-solve is fragile in Rhino 7 GH)
"""

import os
import math
import xml.etree.ElementTree as ET

# RhinoCommon
import Rhino
from Rhino.Geometry import Point3d, Vector3d, Plane, Transform, Line, Sphere
import System
import scriptcontext as sc

#############################
# Utilities: 4x4 matrices   #
#############################

def mat_identity():
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def mat_mul(A, B):
    # 4x4 * 4x4 (column-vector convention)
    C = [[0.0]*4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            s = 0.0
            for k in range(4):
                s += A[i][k] * B[k][j]
            C[i][j] = s
    return C


def mat_apply(A, p):
    # Apply to a 3D point (as column vector with homogeneous = 1)
    x, y, z = p
    v = [x, y, z, 1.0]
    r = [0.0, 0.0, 0.0, 0.0]
    for i in range(4):
        s = 0.0
        for k in range(4):
            s += A[i][k] * v[k]
        r[i] = s
    if r[3] != 0.0:
        invw = 1.0 / r[3]
        return [r[0]*invw, r[1]*invw, r[2]*invw]
    return [r[0], r[1], r[2]]


def mat_apply_vec3(A, v):
    # Apply to a direction vector (ignore translation)
    x, y, z = v
    r = [
        A[0][0]*x + A[0][1]*y + A[0][2]*z,
        A[1][0]*x + A[1][1]*y + A[1][2]*z,
        A[2][0]*x + A[2][1]*y + A[2][2]*z,
    ]
    return r


def mat_trans(x, y, z):
    M = mat_identity()
    M[0][3] = float(x)
    M[1][3] = float(y)
    M[2][3] = float(z)
    return M


def mat_rot_x(a):
    c = math.cos(a); s = math.sin(a)
    return [
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0, 0, 1],
    ]


def mat_rot_y(a):
    c = math.cos(a); s = math.sin(a)
    return [
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1],
    ]


def mat_rot_z(a):
    c = math.cos(a); s = math.sin(a)
    return [
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1],
    ]


def mat_rot_rpy(roll, pitch, yaw):
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    Rz = mat_rot_z(yaw)
    Ry = mat_rot_y(pitch)
    Rx = mat_rot_x(roll)
    return mat_mul(mat_mul(Rz, Ry), Rx)


def mat_rot_axis(axis, a):
    # Rodrigues rotation in joint-local frame
    ax, ay, az = axis
    n = math.sqrt(ax*ax + ay*ay + az*az)
    if n == 0.0:
        return mat_identity()
    x = ax/n; y = ay/n; z = az/n
    c = math.cos(a); s = math.sin(a); C = 1.0 - c
    R = [
        [x*x*C + c,   x*y*C - z*s, x*z*C + y*s, 0],
        [y*x*C + z*s, y*y*C + c,   y*z*C - x*s, 0],
        [z*x*C - y*s, z*y*C + x*s, z*z*C + c,   0],
        [0, 0, 0, 1],
    ]
    return R


def mat_to_rhino_transform(M):
    # Convert 4x4 (row-major) to Rhino Transform
    T = Transform(1.0)
    # Rhino Transform is row-major: Mij into M[i, j]
    for i in range(4):
        for j in range(4):
            T[i, j] = M[i][j]
    return T


def transform_to_plane(M):
    # Build a Plane from a 4x4 transform
    o = mat_apply(M, (0, 0, 0))
    x = mat_apply_vec3(M, (1, 0, 0))
    y = mat_apply_vec3(M, (0, 1, 0))
    ox = Point3d(o[0], o[1], o[2])
    vx = Vector3d(x[0], x[1], x[2])
    vy = Vector3d(y[0], y[1], y[2])
    # Ensure non-degenerate axes
    if vx.IsZero:
        vx = Vector3d(1, 0, 0)
    if vy.IsZero:
        vy = Vector3d(0, 1, 0)
    try:
        pl = Plane(ox, vx, vy)
    except:
        pl = Plane.WorldXY
        pl.Origin = ox
    return pl

#############################
# URDF parsing              #
#############################

def parse_float_list(s):
    if not s:
        return []
    return [float(v) for v in s.strip().split()]


def load_urdf(path):
    if not os.path.isfile(path):
        raise IOError("URDF file not found: {0}".format(path))
    tree = ET.parse(path)
    root = tree.getroot()
    # Gather links
    links = set()
    for ln in root.findall('link'):
        nm = ln.get('name')
        if nm:
            links.add(nm)
    # Gather joints
    joints = []
    for jn in root.findall('joint'):
        jd = {}
        jd['name'] = jn.get('name')
        jd['type'] = jn.get('type')
        p = jn.find('parent')
        c = jn.find('child')
        jd['parent'] = p.get('link') if p is not None else None
        jd['child'] = c.get('link') if c is not None else None
        # origin
        org = jn.find('origin')
        xyz = (0.0, 0.0, 0.0)
        rpy = (0.0, 0.0, 0.0)
        if org is not None:
            if org.get('xyz'):
                f = parse_float_list(org.get('xyz'))
                if len(f) == 3:
                    xyz = tuple(f)
            if org.get('rpy'):
                f = parse_float_list(org.get('rpy'))
                if len(f) == 3:
                    rpy = tuple(f)
        jd['origin_xyz'] = xyz
        jd['origin_rpy'] = rpy
        # axis
        ax = (1.0, 0.0, 0.0)  # URDF default
        axn = jn.find('axis')
        if axn is not None and axn.get('xyz'):
            f = parse_float_list(axn.get('xyz'))
            if len(f) == 3:
                ax = tuple(f)
        jd['axis'] = ax
        # limits (optional)
        lim = jn.find('limit')
        if lim is not None:
            lower = lim.get('lower'); upper = lim.get('upper')
            jd['limit_lower'] = float(lower) if lower is not None else None
            jd['limit_upper'] = float(upper) if upper is not None else None
        else:
            jd['limit_lower'] = None
            jd['limit_upper'] = None
        joints.append(jd)
    return links, joints


def build_graph(links, joints):
    parent_to_children = {}
    child_to_joint = {}
    for jd in joints:
        parent = jd['parent']
        child = jd['child']
        if parent not in parent_to_children:
            parent_to_children[parent] = []
        parent_to_children[parent].append(child)
        child_to_joint[child] = jd
    # roots: links that are never child
    children = set(child_to_joint.keys())

    roots = [ln for ln in links if ln not in children]
    return parent_to_children, child_to_joint, roots

#############################
# Visuals & Mesh binding    #
#############################

def mat_inv_rigid(M):
    # Inverse of a rigid transform (R|t; 0 0 0 1)
    Rt = [
        [M[0][0], M[1][0], M[2][0], 0.0],
        [M[0][1], M[1][1], M[2][1], 0.0],
        [M[0][2], M[1][2], M[2][2], 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    t = [M[0][3], M[1][3], M[2][3]]
    t_inv = [- (Rt[0][0]*t[0] + Rt[0][1]*t[1] + Rt[0][2]*t[2]),
             - (Rt[1][0]*t[0] + Rt[1][1]*t[1] + Rt[1][2]*t[2]),
             - (Rt[2][0]*t[0] + Rt[2][1]*t[1] + Rt[2][2]*t[2])]
    Rt[0][3] = t_inv[0]
    Rt[1][3] = t_inv[1]
    Rt[2][3] = t_inv[2]
    return Rt


def load_visuals(path):
    tree = ET.parse(path)
    root = tree.getroot()
    visuals = {}
    for ln in root.findall('link'):
        name = ln.get('name')
        if not name:
            continue
        v = ln.find('visual')
        if v is None:
            continue
        org = v.find('origin')
        v_xyz = (0.0, 0.0, 0.0)
        v_rpy = (0.0, 0.0, 0.0)
        if org is not None:
            if org.get('xyz'):
                f = parse_float_list(org.get('xyz'))
                if len(f) == 3:
                    v_xyz = tuple(f)
            if org.get('rpy'):
                f = parse_float_list(org.get('rpy'))
                if len(f) == 3:
                    v_rpy = tuple(f)
        geom = v.find('geometry')
        if geom is None:
            continue
        ms = geom.find('mesh')
        if ms is None:
            continue
        fn = ms.get('filename')
        if not fn:
            continue
        sca = (1.0, 1.0, 1.0)
        if ms.get('scale'):
            f = parse_float_list(ms.get('scale'))
            if len(f) == 3:
                sca = (float(f[0]), float(f[1]), float(f[2]))
        visuals[name] = {
            'filename': fn,
            'origin_xyz': v_xyz,
            'origin_rpy': v_rpy,
            'scale': sca,
        }
    return visuals


def ensure_parent_layer():
    doc = Rhino.RhinoDoc.ActiveDoc
    lyr = doc.Layers.FindName('URDF')
    if lyr:
        return lyr.Index
    import Rhino.DocObjects as rd
    new_layer = rd.Layer()
    new_layer.Name = 'URDF'
    idx = doc.Layers.Add(new_layer)
    return idx


def ensure_link_layer(link_name):
    doc = Rhino.RhinoDoc.ActiveDoc
    parent_idx = ensure_parent_layer()
    full_path = 'URDF::' + link_name
    idx = doc.Layers.FindByFullPath(full_path, -1)
    if idx >= 0:
        return idx
    import Rhino.DocObjects as rd
    lyr = rd.Layer()
    lyr.Name = link_name
    lyr.ParentLayerId = doc.Layers[parent_idx].Id
    idx = doc.Layers.Add(lyr)
    return idx


def comp_guid():
    try:
        return str(ghenv.Component.InstanceGuid)
    except:
        return 'GLOBAL'


def abs_mesh_path(base_dir, mesh_filename):
    p = mesh_filename
    if not os.path.isabs(p):
        p = os.path.normpath(os.path.join(base_dir, p))
    return p


def import_and_place_mesh(mesh_path, layer_idx, scale_tuple, world_M):
    doc = Rhino.RhinoDoc.ActiveDoc
    before = set([obj.Id for obj in doc.Objects])
    # Switch current layer
    doc.Layers.SetCurrentLayerIndex(layer_idx, True)
    cmd = '-_Import "{0}" _Enter'.format(mesh_path.replace('\\', '/'))
    Rhino.RhinoApp.RunScript(cmd, False)
    after = [obj for obj in doc.Objects]
    new_ids = [obj.Id for obj in after if obj.Id not in before and obj.Attributes.LayerIndex == layer_idx]
    # Apply scale (about 0,0,0 in file coords)
    sx, sy, sz = scale_tuple
    if abs(sx-1.0) > 1e-9 or abs(sy-1.0) > 1e-9 or abs(sz-1.0) > 1e-9:
        S = Transform.Scale(Point3d(0,0,0), sx, sy, sz)
        for gid in new_ids:
            doc.Objects.Transform(gid, S, True)
    # Apply initial placement
    T = mat_to_rhino_transform(world_M)
    for gid in new_ids:
        doc.Objects.Transform(gid, T, True)
    return new_ids


def ensure_mesh_binding(URDFPath, base_dir, visuals, T_link):
    key = 'URDF_MAP_' + comp_guid()
    mm = sc.sticky.get(key, None)
    if mm is None:
        mm = {}
    count = 0
    for link, vis in visuals.items():
        fn = abs_mesh_path(base_dir, vis['filename'])
        if not os.path.isfile(fn):
            continue
        if link in mm and 'ids' in mm[link]:
            continue  # already bound
        layer_idx = ensure_link_layer(link)
        Tvis = compose_origin(vis['origin_xyz'], vis['origin_rpy'])
        M_world = mat_mul(T_link.get(link, mat_identity()), Tvis)
        ids = import_and_place_mesh(fn, layer_idx, vis['scale'], M_world)
        if ids:
            mm[link] = {'ids': ids, 'last': M_world}
            count += len(ids)
    sc.sticky[key] = mm
    return count


def update_mesh_binding(visuals, T_link):
    key = 'URDF_MAP_' + comp_guid()
    mm = sc.sticky.get(key, None)
    if not mm:
        return 0
    doc = Rhino.RhinoDoc.ActiveDoc
    moved = 0
    for link, entry in mm.items():
        if 'ids' not in entry or link not in visuals:
            continue
        Tvis = compose_origin(visuals[link]['origin_xyz'], visuals[link]['origin_rpy'])
        M_new = mat_mul(T_link.get(link, mat_identity()), Tvis)
        M_prev = entry.get('last', mat_identity())
        Delta = mat_mul(mat_inv_rigid(M_prev), M_new)
        Td = mat_to_rhino_transform(Delta)
        for gid in entry['ids']:
            doc.Objects.Transform(gid, Td, True)
            moved += 1
        entry['last'] = M_new
    sc.sticky[key] = mm
    return moved



#############################
# FK computation            #
#############################

def joint_value_map(joints, J_vals_rad):
    # Map six provided values to joint_1..joint_6; others default 0
    m = {}
    idx = 0
    for jd in joints:
        nm = jd['name']
        if nm and nm.startswith('joint_'):
            try:
                # e.g., joint_3 -> 3
                k = int(nm.split('_')[-1])
                if 1 <= k <= 6 and k-1 < len(J_vals_rad):
                    m[nm] = float(J_vals_rad[k-1])
            except:
                pass
    # fill others as 0
    for jd in joints:
        nm = jd['name']
        if nm not in m:
            m[nm] = 0.0
    return m


def compose_origin(xyz, rpy):
    Tx = mat_trans(xyz[0], xyz[1], xyz[2])
    R = mat_rot_rpy(rpy[0], rpy[1], rpy[2])
    return mat_mul(Tx, R)


def fk_all(links, joints, Jmap):
    parent_to_children, child_to_joint, roots = build_graph(links, joints)
    T_link = {}
    # initialize roots as identity
    for r in roots:
        T_link[r] = mat_identity()
    # BFS/DFS over graph
    stack = list(roots)
    visited = set()
    order = []
    while stack:
        parent = stack.pop(0)
        order.append(parent)
        visited.add(parent)
        children = parent_to_children.get(parent, [])
        for child in children:
            jd = child_to_joint[child]
            oxyz = jd['origin_xyz']
            orpy = jd['origin_rpy']
            J = compose_origin(oxyz, orpy)
            if jd['type'] == 'revolute' or jd['type'] == 'continuous':
                a = float(Jmap.get(jd['name'], 0.0))
                Rj = mat_rot_axis(jd['axis'], a)
                J = mat_mul(J, Rj)
            elif jd['type'] == 'prismatic':
                # translate along axis by q (meters)
                a = float(Jmap.get(jd['name'], 0.0))
                ax = jd['axis']
                J = mat_mul(J, mat_trans(ax[0]*a, ax[1]*a, ax[2]*a))
            # else fixed: just origin
            T_parent = T_link.get(jd['parent'], mat_identity())
            T_child = mat_mul(T_parent, J)
            T_link[child] = T_child
            if child not in visited:
                stack.append(child)
    return T_link, order

#############################
# GH entry                  #
#############################

def ensure_inputs():
    # Set defaults if running outside GH
    global URDFPath, URDFDir, J, Deg, AxisLen, LoadMeshes, UpdateMeshes
    try:
        URDFPath
    except:
        URDFPath = ''
    try:
        URDFDir
    except:
        URDFDir = ''
    try:
        J
    except:
        J = [0, 0, 0, 0, 0, 0]
    try:
        Deg
    except:
        Deg = True
    try:
        AxisLen
    except:
        AxisLen = 100.0
    try:
        LoadMeshes
    except:
        LoadMeshes = False
    try:
        UpdateMeshes
    except:
        UpdateMeshes = True


def build_preview(planes, axis_len):
    geos = []
    L = float(axis_len)
    if L <= 0.0:
        L = 100.0
    for pl in planes:
        o = pl.Origin
        x = pl.XAxis; x.Unitize();
        y = pl.YAxis; y.Unitize();
        z = pl.ZAxis; z.Unitize();
        geos.append(Line(o, o + x*L).ToNurbsCurve())
        geos.append(Line(o, o + y*L).ToNurbsCurve())
        geos.append(Line(o, o + z*L).ToNurbsCurve())
        geos.append(Sphere(o, L*0.03).ToBrep())
    return geos


def main():
    ensure_inputs()

    # Resolve URDF path from file or directory
    URDFPath_local = URDFPath
    if (not URDFPath_local) and URDFDir:
        try:
            for nm in os.listdir(URDFDir):
                if nm.lower().endswith('.urdf'):
                    URDFPath_local = os.path.join(URDFDir, nm)
                    break
        except:
            URDFPath_local = ''

    if not URDFPath_local:
        Debug = 'Please provide URDFPath or URDFDir containing a .urdf file.'
        return [], [], [], [], Plane.WorldXY, [], Debug

    try:
        links, joints = load_urdf(URDFPath_local)
        visuals = load_visuals(URDFPath_local)
    except Exception as e:
        Debug = 'Failed to load URDF: {0}'.format(e)
        return [], [], [], [], Plane.WorldXY, [], Debug

    base_dir = URDFDir if (URDFDir and os.path.isdir(URDFDir)) else os.path.dirname(URDFPath_local)

    # Prepare joint values
    J_vals = list(J) if J is not None else [0,0,0,0,0,0]
    while len(J_vals) < 6:
        J_vals.append(0.0)
    if Deg:
        J_rad = [math.radians(v) for v in J_vals]
    else:
        J_rad = [float(v) for v in J_vals]

    Jmap = joint_value_map(joints, J_rad)
    T_link, order = fk_all(links, joints, Jmap)

    # Optional: bind/import meshes once, and update each solve
    bound = 0
    if LoadMeshes:
        bound = ensure_mesh_binding(URDFPath_local, base_dir, visuals, T_link)
    moved = 0
    if UpdateMeshes:
        moved = update_mesh_binding(visuals, T_link)

    # Build outputs
    link_names = list(order)
    link_planes = []
    for ln in link_names:
        M = T_link.get(ln, mat_identity())
        link_planes.append(transform_to_plane(M))

    # Joint planes (at parent->joint origin, before joint rotation) for named joints
    joint_names = []
    joint_planes = []
    for jd in joints:
        nm = jd['name']
        joint_names.append(nm)
        T_parent = T_link.get(jd['parent'], mat_identity())
        Jorg = compose_origin(jd['origin_xyz'], jd['origin_rpy'])
        T_jointframe = mat_mul(T_parent, Jorg)
        joint_planes.append(transform_to_plane(T_jointframe))

    # Pick EEF plane
    eef_name = None
    for candidate in ('tool0', 'flange', 'link_6'):
        if candidate in T_link:
            eef_name = candidate
            break
    if eef_name is None and len(link_names) > 0:
        eef_name = link_names[-1]
    eef_plane = transform_to_plane(T_link.get(eef_name, mat_identity()))

    preview = build_preview(link_planes, AxisLen)

    Debug = 'Loaded URDF: {0}\nBaseDir: {1}\nLinks: {2}\nJoints: {3}\nVisuals: {4}\nBound: {5}, Moved: {6}\nEEF: {7}'.format(
        os.path.basename(URDFPath_local), base_dir, len(link_names), len(joint_names), len(visuals), bound, moved, eef_name)

    return joint_names, link_names, joint_planes, link_planes, eef_plane, preview, Debug

# Execute for GH
try:
    JointNames, LinkNames, JointPlanes, LinkPlanes, EEFPlane, Preview, Debug = main()
except Exception as _err:
    # Graceful failure with message for GH
    JointNames = []
    LinkNames = []
    JointPlanes = []
    LinkPlanes = []
    EEFPlane = Plane.WorldXY
    Preview = []
    Debug = 'Exception: {0}'.format(_err)

