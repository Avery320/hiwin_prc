# ========== 電池02：URDF 運動學 ==========
# 輸入：URDFPath (str), Meshes (list), MeshPaths (list), J (list), Deg (bool), Reload (bool)
# 輸出：G, Names, JointOrder

import sys
sys.path.append(r"/Users/avery_tsai/project/hiwin_prc")
from robot import urdf_loader

# 載入 URDF 並計算運動學
G, Names, JointOrder = urdf_loader.load(
    URDFPath if 'URDFPath' in globals() and URDFPath else r"/Users/avery_tsai/project/hiwin_prc/urdf/walker_arm/urdf/walker_arm.urdf",
    Meshes if 'Meshes' in globals() else [],
    MeshPaths if 'MeshPaths' in globals() else [],
    J if 'J' in globals() and J else [0, 0, 0, 0, 0, 0],
    Deg if 'Deg' in globals() else True,
    Reload if 'Reload' in globals() else False
)

a = G

