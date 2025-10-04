"""
mesh_loader.py - 電池01：網格檔案載入器

功能：
    從資料夾載入網格檔案（.stl, .obj, .dae 等）到 Grasshopper
    自動合併同一檔案中的多個 mesh 物件
    支援快取機制以提升效能

作者：Avery Tsai
版本：2.0
日期：2025-10-05
"""

import os

try:
    import Rhino
    import scriptcontext as sc
    from Rhino.Geometry import Mesh
except ImportError:
    Rhino = sc = Mesh = None

CACHE_KEY = 'GH_ROBOT_MESH_CACHE'

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

def find_mesh_files(dirpath):
    """遞迴搜尋資料夾中的 .stl 檔案"""
    if not dirpath or not os.path.isdir(dirpath):
        return []

    files = []
    for root, _, filenames in os.walk(dirpath):
        for fn in filenames:
            if fn.lower().endswith('.stl'):
                files.append(os.path.abspath(os.path.join(root, fn)))

    return sorted(files)

def import_mesh_file(filepath):
    """匯入單一 .stl 檔案並合併所有 mesh"""
    if not filepath or not os.path.exists(filepath):
        return None

    doc = Rhino.RhinoDoc.ActiveDoc
    if not doc:
        return None

    cmd = '-_Import "{}" _Enter'.format(filepath.replace('"', '\\"'))
    prev_redraw = doc.Views.RedrawEnabled
    doc.Views.RedrawEnabled = False

    try:
        before_ids = set(obj.Id for obj in doc.Objects)
        Rhino.RhinoApp.RunScript(cmd, False)
        after_objs = [obj for obj in doc.Objects if obj.Id not in before_ids]

        # 提取並合併 mesh
        meshes = []
        for obj in after_objs:
            geo = obj.Geometry
            if isinstance(geo, Mesh):
                meshes.append(geo.DuplicateMesh())
            else:
                try:
                    brep_meshes = Rhino.Geometry.Mesh.CreateFromBrep(geo)
                    if brep_meshes:
                        meshes.extend(brep_meshes)
                except:
                    pass
            doc.Objects.Delete(obj, True)

        # 合併
        if not meshes:
            return None
        if len(meshes) == 1:
            return meshes[0]

        merged = Mesh()
        for m in meshes:
            merged.Append(m)
        return merged

    finally:
        doc.Views.RedrawEnabled = prev_redraw

def load_meshes(dirpath, recursive, extensions, use_cache):
    """批次載入資料夾中的所有 .stl 檔案"""
    cache = _get_cache()
    files = find_mesh_files(dirpath)

    meshes = []
    paths = []

    for fpath in files:
        cache_key = ('MESH', fpath, os.path.getmtime(fpath))

        if cache_key in cache:
            mesh = cache[cache_key].DuplicateMesh()
        else:
            mesh = import_mesh_file(fpath)
            if mesh:
                cache[cache_key] = mesh.DuplicateMesh()

        if mesh:
            meshes.append(mesh)
            paths.append(fpath)

    return meshes, paths

def load(dirpath, reload=False):
    """載入網格檔案（主要 API）

    自動遞迴搜尋 .stl 檔案，每個檔案會被合併成一個 mesh

    Args:
        dirpath: 搜尋目錄
        reload: 強制重新載入，清除快取（預設 False）

    Returns:
        (meshes, paths): Mesh 物件列表與對應的檔案路徑列表
    """
    if reload:
        clear_cache()

    if not dirpath or not os.path.isdir(dirpath):
        return [], []

    meshes, paths = load_meshes(dirpath, True, "stl", not reload)
    return meshes, paths


# 向後相容的別名
mesh_loader = load
