# 更新日誌 - 2025-10-05

## ✅ 完成項目

### 1. 新增 Reload 參數

#### mesh_loader.py
- ✅ 新增 `reload` 參數（預設 False）
- ✅ 當 `reload=True` 時自動清除快取
- ✅ API: `load(dirpath, reload=False)`

#### urdf_loader.py
- ✅ 新增 `reload` 參數（預設 False）
- ✅ 當 `reload=True` 時自動清除快取
- ✅ API: `load(urdf_path, meshes, mesh_paths, joint_values=None, use_degrees=True, reload=False)`

### 2. 更新範例程式碼

#### gh_mesh_loader.py
```python
Meshes, Paths = mesh_loader.load(
    DirPath if 'DirPath' in globals() and DirPath else r"/Users/avery_tsai/project/hiwin_prc/urdf/walker_arm",
    Reload if 'Reload' in globals() else False
)
```

#### gh_urdf_loader.py
```python
G, Names, JointOrder = urdf_loader.load(
    URDFPath if 'URDFPath' in globals() and URDFPath else r"/Users/avery_tsai/project/hiwin_prc/urdf/walker_arm/urdf/walker_arm.urdf",
    Meshes if 'Meshes' in globals() else [],
    MeshPaths if 'MeshPaths' in globals() else [],
    J if 'J' in globals() and J else [0, 0, 0, 0, 0, 0],
    Deg if 'Deg' in globals() else True,
    Reload if 'Reload' in globals() else False
)
```

### 3. 更新 README.md

- ✅ 更新檔案名稱（gh_battery01_example.py → gh_mesh_loader.py）
- ✅ 更新電池01輸入參數說明（新增 Reload）
- ✅ 更新電池02輸入參數說明（新增 Reload）
- ✅ 更新範例程式碼
- ✅ 更新 Grasshopper 連接圖
- ✅ 更新常見問題說明
- ✅ 移除已刪除的參數說明（Scale, URDFRoot）

---

## 📊 API 總結

### 電池01：mesh_loader.load()

**參數**：
- `dirpath` (str): 網格資料夾路徑
- `reload` (bool): 強制重新載入，清除快取（預設 False）

**返回**：
- `(meshes, paths)`: 網格列表與路徑列表

**特性**：
- 自動遞迴搜尋 .stl 檔案
- 自動合併同一檔案中的多個 mesh
- 支援快取機制（可用 reload 清除）

### 電池02：urdf_loader.load()

**參數**：
- `urdf_path` (str): URDF 檔案路徑
- `meshes` (list): 預載入的網格列表
- `mesh_paths` (list): 對應的檔案路徑列表
- `joint_values` (list): 六軸關節值 [J1..J6]（預設 [0,0,0,0,0,0]）
- `use_degrees` (bool): True=角度，False=弧度（預設 True）
- `reload` (bool): 強制重新載入，清除快取（預設 False）

**返回**：
- `(meshes, names, joint_order)`: 變換後的網格、Link名稱、關節順序

**特性**：
- 解析 URDF visual geometry
- 計算前向運動學
- 自動處理 xyz, rpy, scale
- 支援快取機制（可用 reload 清除）

---

## 🎯 使用方式

### 在 Grasshopper 中

#### 電池01（2 個輸入）
- `DirPath` (str): 網格資料夾路徑
- `Reload` (bool): 強制重新載入（預設 False）

#### 電池02（6 個輸入）
- `URDFPath` (str): URDF 檔案路徑
- `Meshes` (list): 來自電池01
- `MeshPaths` (list): 來自電池01
- `J` (list): 六軸關節值
- `Deg` (bool): 使用角度（預設 True）
- `Reload` (bool): 強制重新載入（預設 False）

---

## 🔧 何時使用 Reload

### 需要設為 True 的情況：
1. 修改了網格檔案（.stl）
2. 修改了 URDF 檔案
3. 網格顯示不正確，懷疑是快取問題
4. 第一次載入後發現路徑配對錯誤

### 可以保持 False 的情況：
1. 只是調整關節值（J）
2. 檔案沒有修改
3. 需要更快的執行速度

---

## 📝 代碼統計

| 檔案 | 行數 |
|------|------|
| `mesh_loader.py` | 167 行 |
| `urdf_loader.py` | 437 行 |
| `gh_mesh_loader.py` | 16 行 |
| `gh_urdf_loader.py` | 20 行 |
| **總計** | **640 行** |

---

## ✨ 改進點

1. **更靈活的快取控制**：用戶可以選擇何時清除快取
2. **更簡潔的 API**：只保留必要的參數
3. **更清晰的文檔**：更新了所有說明和範例
4. **向後相容**：保留了 `urdf_draw()` 別名

---

**版本**: 2.1  
**日期**: 2025-10-05  
**作者**: Avery Tsai

