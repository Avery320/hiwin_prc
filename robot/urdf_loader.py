"""
urdf_loader.py - 電池02：URDF 運動學引擎

功能：
    模擬 RViz 功能 - 解析 URDF visual geometry，計算前向運動學
    支援六軸機器人的關節控制
    自動處理 URDF 中的 xyz、rpy、scale 參數

作者：Avery Tsai
版本：2.0
日期：2025-10-05
"""

import os
import math
import xml.etree.ElementTree as ET
from collections import deque

try:
    import scriptcontext as sc
    from Rhino.Geometry import Transform, Mesh, Point3d, Vector3d, Plane
except ImportError:
    sc = None
    Transform = Mesh = Point3d = Vector3d = Plane = None

CACHE_KEY = 'GH_ROBOT_URDF_CACHE'

def _parse_floats(s):
    """解析浮點數字串"""
    if not s:
        return []
    return [float(x) for x in s.strip().replace(',', ' ').split() if x]


def _rpy_to_transform(roll, pitch, yaw):
    """URDF RPY 轉換：固定軸旋轉 X(roll) -> Y(pitch) -> Z(yaw)"""
    origin = Point3d(0, 0, 0)
    Rx = Transform.Rotation(roll, Vector3d(1, 0, 0), origin)
    Ry = Transform.Rotation(pitch, Vector3d(0, 1, 0), origin)
    Rz = Transform.Rotation(yaw, Vector3d(0, 0, 1), origin)

    T = Transform.Identity
    T = Transform.Multiply(T, Rz)
    T = Transform.Multiply(T, Ry)
    T = Transform.Multiply(T, Rx)
    return T


def _xyzrpy_to_transform(xyz, rpy):
    """xyz + rpy 轉換為 Transform"""
    T_trans = Transform.Translation(Vector3d(*xyz)) if xyz else Transform.Identity
    T_rot = _rpy_to_transform(*rpy) if rpy else Transform.Identity
    return Transform.Multiply(T_trans, T_rot)


def _axis_angle_transform(axis, angle):
    """軸角旋轉轉換"""
    ax = Vector3d(*axis)
    if ax.IsZero:
        return Transform.Identity
    ax.Unitize()
    return Transform.Rotation(angle, ax, Point3d(0, 0, 0))


def _normalize_path(path):
    """標準化路徑（用於比對）"""
    if not path:
        return ''
    return os.path.normpath(os.path.abspath(path)).lower()


def _get_cache():
    """取得快取字典"""
    if sc is None:
        return {}
    cache = sc.sticky.get(CACHE_KEY)
    if cache is None:
        cache = {}
        sc.sticky[CACHE_KEY] = cache
    return cache


def clear_cache():
    """清除所有快取"""
    if sc:
        sc.sticky.pop(CACHE_KEY, None)

class Visual(object):
    """URDF <visual> 元素"""
    __slots__ = ('mesh_path', 'xyz', 'rpy', 'scale')

    def __init__(self, mesh_path, xyz, rpy, scale):
        self.mesh_path = mesh_path  # 網格檔案絕對路徑
        self.xyz = xyz              # [x, y, z]
        self.rpy = rpy              # [roll, pitch, yaw]
        self.scale = scale          # [sx, sy, sz]


class Link(object):
    """URDF <link> 元素"""
    __slots__ = ('name', 'visuals')

    def __init__(self, name):
        self.name = name
        self.visuals = []


class Joint(object):
    """URDF <joint> 元素"""
    __slots__ = ('name', 'type', 'parent', 'child', 'xyz', 'rpy', 'axis')

    def __init__(self, name, jtype, parent, child, xyz, rpy, axis):
        self.name = name
        self.type = jtype           # revolute, continuous, prismatic, fixed
        self.parent = parent        # parent link name
        self.child = child          # child link name
        self.xyz = xyz              # [x, y, z]
        self.rpy = rpy              # [roll, pitch, yaw]
        self.axis = axis or [0, 0, 1]  # [x, y, z]


class RobotModel(object):
    """URDF 機器人模型"""
    __slots__ = ('links', 'joints', 'root', 'joint_order')

    def __init__(self):
        self.links = {}         # {name: Link}
        self.joints = {}        # {name: Joint}
        self.root = None        # root link name
        self.joint_order = []   # 可動關節名稱列表（樹狀順序）

    def resolve_root(self):
        """找出 root link（沒有 parent 的 link）"""
        children = {j.child for j in self.joints.values()}
        candidates = [name for name in self.links if name not in children]
        self.root = candidates[0] if candidates else None

    def build_order(self):
        """建立可動關節的順序（BFS 遍歷）"""
        if not self.root:
            self.resolve_root()

        movable = []
        queue = deque([self.root])
        visited = set()

        while queue:
            link = queue.popleft()
            if link in visited:
                continue
            visited.add(link)

            for joint in self.joints.values():
                if joint.parent == link:
                    if joint.type in ('revolute', 'continuous', 'prismatic'):
                        movable.append(joint.name)
                    queue.append(joint.child)

        self.joint_order = movable

def _resolve_mesh_path(filename, base_dir, urdf_root):
    """解析網格檔案路徑

    支援：
    - package:// 路徑
    - 相對路徑（../meshes/...）
    - 絕對路徑
    """
    if not filename:
        return None

    fn = filename.strip()

    # package:// 路徑
    if fn.startswith('package://'):
        rest = fn[10:]  # 移除 'package://'
        sub_path = rest.split('/', 1)[1] if '/' in rest else ''
        full_path = os.path.join(urdf_root, sub_path)
        if os.path.exists(full_path):
            return os.path.abspath(full_path)

    # 相對路徑
    rel_path = os.path.join(base_dir, fn)
    if os.path.exists(rel_path):
        return os.path.abspath(rel_path)

    # 絕對路徑
    if os.path.isabs(fn) and os.path.exists(fn):
        return os.path.abspath(fn)

    return None


def parse_urdf(urdf_path):
    """解析 URDF 檔案（只處理 <visual><geometry>）"""
    cache = _get_cache()
    cache_key = ('URDF', urdf_path)
    if cache_key in cache:
        return cache[cache_key]

    tree = ET.parse(urdf_path)
    root = tree.getroot()
    robot = RobotModel()

    base_dir = os.path.dirname(urdf_path)
    urdf_root = os.path.abspath(os.path.join(base_dir, '..'))

    # 解析 links（只處理 <visual>）
    for link_elem in root.findall('link'):
        link_name = link_elem.get('name')
        link = Link(link_name)

        for visual_elem in link_elem.findall('visual'):
            # 解析 origin
            origin_elem = visual_elem.find('origin')
            xyz = _parse_floats(origin_elem.get('xyz')) if origin_elem is not None else [0, 0, 0]
            rpy = _parse_floats(origin_elem.get('rpy')) if origin_elem is not None else [0, 0, 0]

            # 解析 geometry/mesh
            geom_elem = visual_elem.find('geometry')
            mesh_elem = geom_elem.find('mesh') if geom_elem is not None else None

            if mesh_elem is not None:
                mesh_file = mesh_elem.get('filename')
                mesh_path = _resolve_mesh_path(mesh_file, base_dir, urdf_root) if mesh_file else None

                # 解析 scale（預設 [1, 1, 1]）
                scale = _parse_floats(mesh_elem.get('scale')) or [1.0, 1.0, 1.0]
                scale = (scale + [1.0, 1.0, 1.0])[:3]  # 確保長度為 3

                link.visuals.append(Visual(mesh_path, xyz, rpy, scale))

        robot.links[link_name] = link

    # 解析 joints
    for joint_elem in root.findall('joint'):
        joint_name = joint_elem.get('name')
        joint_type = joint_elem.get('type')
        parent_link = joint_elem.find('parent').get('link')
        child_link = joint_elem.find('child').get('link')

        # 解析 origin
        origin_elem = joint_elem.find('origin')
        xyz = _parse_floats(origin_elem.get('xyz')) if origin_elem is not None else [0, 0, 0]
        rpy = _parse_floats(origin_elem.get('rpy')) if origin_elem is not None else [0, 0, 0]

        # 解析 axis
        axis_elem = joint_elem.find('axis')
        axis = _parse_floats(axis_elem.get('xyz')) if axis_elem is not None else [0, 0, 1]

        joint = Joint(joint_name, joint_type, parent_link, child_link, xyz, rpy, axis)
        robot.joints[joint_name] = joint

    # 建立機器人樹結構
    robot.resolve_root()
    robot.build_order()

    # 快取結果
    cache[cache_key] = robot
    return robot


# ==================== 前向運動學 ====================


# -------------------- Kinematics --------------------

def compute_link_transforms(robot, joint_values):
    """計算所有 link 的世界座標變換（前向運動學）

    Args:
        robot: RobotModel 實例
        joint_values: dict {joint_name: value_in_radians}

    Returns:
        dict {link_name: Transform}
    """
    T_world = {}
    if not robot.root:
        return T_world

    # Root link 的變換為單位矩陣
    T_world[robot.root] = Transform.Identity

    # 建立 parent -> children joints 的映射
    children_joints = {}
    for joint in robot.joints.values():
        children_joints.setdefault(joint.parent, []).append(joint)

    def traverse(link_name):
        """遞迴遍歷機器人樹"""
        for joint in children_joints.get(link_name, []):
            # 取得 parent link 的變換
            T_parent = T_world[link_name]

            # 套用 joint origin 變換（xyz + rpy）
            T_joint_origin = _xyzrpy_to_transform(joint.xyz, joint.rpy)
            T = Transform.Multiply(T_parent, T_joint_origin)

            # 套用關節運動（revolute/continuous/prismatic）
            joint_value = joint_values.get(joint.name, 0.0)
            if joint.type in ('revolute', 'continuous'):
                # 旋轉關節：繞 axis 旋轉 joint_value 弧度
                T_rotation = _axis_angle_transform(joint.axis, joint_value)
                T = Transform.Multiply(T, T_rotation)
            elif joint.type == 'prismatic':
                # 平移關節：沿 axis 平移 joint_value 距離
                axis_vec = Vector3d(*joint.axis)
                axis_vec.Unitize()
                T_translation = Transform.Translation(axis_vec * joint_value)
                T = Transform.Multiply(T, T_translation)
            # fixed joint 不需要額外變換

            # 儲存 child link 的變換
            T_world[joint.child] = T

            # 遞迴處理子節點
            traverse(joint.child)

    traverse(robot.root)
    return T_world

def normalize_path(p):
    """Normalize path for comparison"""
    if not p:
        return ''
    return os.path.abspath(p).lower().replace('\\', '/')


def match_meshes(robot, input_meshes, input_paths):
    """將網格與 URDF visual geometry 配對"""
    path_to_mesh = {}
    for i, path in enumerate(input_paths or []):
        if i < len(input_meshes):
            path_to_mesh[normalize_path(path)] = input_meshes[i]

    link_meshes = {}
    for link_name, link in robot.links.items():
        link_meshes[link_name] = []
        for visual in link.visuals:
            mesh = path_to_mesh.get(normalize_path(visual.mesh_path))
            if mesh:
                link_meshes[link_name].append((mesh, visual))

    return link_meshes


def assemble_geometry(robot, T_links, link_meshes):
    """組裝機器人幾何（套用變換：scale → visual origin → link transform）"""
    out_meshes = []
    out_names = []

    for link_name in robot.links.keys():
        T_link = T_links.get(link_name, Transform.Identity)

        for mesh, visual in link_meshes.get(link_name, []):
            m = mesh.DuplicateMesh()

            # 1. Scale
            if visual.scale != [1.0, 1.0, 1.0]:
                m.Transform(Transform.Scale(Plane.WorldXY, visual.scale[0], visual.scale[1], visual.scale[2]))

            # 2. Visual origin
            m.Transform(_xyzrpy_to_transform(visual.xyz, visual.rpy))

            # 3. Link transform
            m.Transform(T_link)

            out_meshes.append(m)
            out_names.append(link_name)

    return out_meshes, out_names

def load(urdf_path, meshes, mesh_paths, joint_values=None, use_degrees=True, reload=False):
    """載入 URDF 並計算運動學

    Args:
        urdf_path: URDF 檔案路徑
        meshes: 預載入的網格列表
        mesh_paths: 對應的檔案路徑列表
        joint_values: 六軸關節值 [J1..J6]（預設 [0,0,0,0,0,0]）
        use_degrees: True=角度，False=弧度（預設 True）
        reload: 強制重新載入，清除快取（預設 False）

    Returns:
        (meshes, names, joint_order): 變換後的網格、Link名稱、關節順序
    """
    if reload:
        clear_cache()

    if not urdf_path or not os.path.exists(urdf_path):
        return [], [], []

    try:
        robot = parse_urdf(urdf_path)
    except:
        return [], [], []

    # 處理關節值
    j_vals = list(joint_values) if joint_values else [0.0] * 6
    j_vals = (j_vals + [0.0] * 6)[:6]

    if use_degrees:
        j_vals = [math.radians(v) for v in j_vals]

    # 建立關節值字典
    joint_dict = {}
    for i, jname in enumerate(robot.joint_order):
        joint_dict[jname] = j_vals[i] if i < 6 else 0.0

    # 計算運動學
    T_links = compute_link_transforms(robot, joint_dict)
    link_meshes = match_meshes(robot, meshes or [], mesh_paths or [])
    result_meshes, names = assemble_geometry(robot, T_links, link_meshes)

    return result_meshes, names, robot.joint_order[:6]


# 向後相容
def urdf_draw(URDFPath, Meshes, MeshPaths, J=None, Deg=True, Reload=False):
    return load(URDFPath, Meshes, MeshPaths, J, Deg, Reload)

