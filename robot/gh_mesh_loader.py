# ========== 電池01：載入網格 ==========
# 輸入：DirPath (str), Reload (bool)
# 輸出：Meshes, Paths

import sys
sys.path.append(r"/Users/avery_tsai/project/hiwin_prc")
from robot import mesh_loader

# 載入網格（自動遞迴搜尋 .stl 檔案）
Meshes, Paths = mesh_loader.load(
    DirPath if 'DirPath' in globals() and DirPath else r"/Users/avery_tsai/project/hiwin_prc/urdf/walker_arm",
    Reload if 'Reload' in globals() else False
)

a = Meshes

