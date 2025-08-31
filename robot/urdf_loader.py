
import os
import xml.etree.ElementTree as ET
from stl import mesh as stl_mesh
import Rhino

# 設定 URDF 路徑（請根據你的 Mac 路徑調整）
urdf_path = r"/Users/avery/OrbStack/docker/volumes/walker_arm/src/walker_arm/hiwin_description/urdf/walker_arm.urdf"
package_root = os.path.dirname(os.path.dirname(os.path.dirname(urdf_path)))  # .../hiwin_description

def resolve_package_uri(uri):
    if uri.startswith("package://hiwin_description/"):
        rel_path = uri.replace("package://hiwin_description/", "")
        return os.path.join(package_root, rel_path)
    return uri

def load_urdf_links(urdf_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    links = []
    for link in root.findall("link"):
        name = link.get("name")
        visual = link.find("visual")
        if visual is not None:
            geometry = visual.find("geometry")
            mesh = geometry.find("mesh")
            if mesh is not None:
                mesh_file = resolve_package_uri(mesh.get("filename"))
                links.append((name, mesh_file))
    return links

# 讀取 STL 並轉為 RhinoCommon Mesh
def stl_to_rhino_mesh(stl_path, scale=1.0):
    if not os.path.exists(stl_path):
        return None
    your_mesh = stl_mesh.Mesh.from_file(stl_path)
    rhino_mesh = Rhino.Geometry.Mesh()
    for v in your_mesh.vectors:
        idx = []
        for pt in v:
            idx.append(rhino_mesh.Vertices.Add(float(pt[0]) * scale, float(pt[1]) * scale, float(pt[2]) * scale))
        rhino_mesh.Faces.AddFace(idx[0], idx[1], idx[2])
    rhino_mesh.Normals.ComputeNormals()
    rhino_mesh.Compact()
    return rhino_mesh

# 主流程：回傳所有 link 的 Rhino Mesh 物件（適用於 Grasshopper）
def load_urdf_meshes(urdf_path, scale=1.0):
    links = load_urdf_links(urdf_path)
    mesh_objs = []
    for name, mesh_path in links:
        m = stl_to_rhino_mesh(mesh_path, scale)
        if m:
            mesh_objs.append(m)
    return mesh_objs

# Grasshopper 輸出
a = load_urdf_meshes(urdf_path, scale=100.0)  # 放大 100 倍