from stl import mesh
import Rhino

stl_path = r"/Users/avery/OrbStack/docker/volumes/walker_arm/src/walker_arm/hiwin_description/meshes/ra610_1476/visual/base.stl"
your_mesh = mesh.Mesh.from_file(stl_path)

scale = 100.0  # 放大 100 倍

rhino_mesh = Rhino.Geometry.Mesh()
for v in your_mesh.vectors:
    idx = []
    for pt in v:
        idx.append(rhino_mesh.Vertices.Add(float(pt[0]) * scale, float(pt[1]) * scale, float(pt[2]) * scale))
    rhino_mesh.Faces.AddFace(idx[0], idx[1], idx[2])

rhino_mesh.Normals.ComputeNormals()
rhino_mesh.Compact()
a = rhino_mesh