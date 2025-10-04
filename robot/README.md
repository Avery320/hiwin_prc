# Grasshopper URDF Robot Viewer 

本專案提供兩個 Grasshopper Python 電池（GhPython 元件），用於在 Grasshopper 中載入並控制 URDF 機器人模型。
支援：rhino7 以上版本

## 檔案位置
- `robot/mesh_loader.py` - 電池01：網格載入器
- `robot/urdf_loader.py` - 電池02：URDF 運動學引擎
- `robot/gh_mesh_loader.py` - 電池01 範例程式碼
- `robot/gh_urdf_loader.py` - 電池02 範例程式碼
- `robot/README.md` - 本說明文件

## 架構概述

```
電池01 (mesh_loader.py)          電池02 (urdf_loader.py)
┌─────────────────────┐         ┌──────────────────────┐
│ 載入 .stl 等網格檔案  │────────>│ 解析 URDF + 運動學     │
│ 輸出 Meshes + Paths │         │ 輸出變換後的機器人模型   │
└─────────────────────┘         └──────────────────────┘
```

## gh_component：mesh_loader.py

### 功能
從指定資料夾載入所有網格檔案（.stl, .obj, .dae 等），轉換為 Grasshopper 可操作的 Mesh 物件。

### 在 Grasshopper 中使用

1. 新增一個 **GhPython** 元件
2. 設定輸入參數（右鍵點擊元件 > Type Hint）：
   - `DirPath` (str): 網格檔案所在資料夾路徑
   - `Reload` (bool): 強制重新載入，清除快取（預設 False）
3. 設定輸出參數：
   - `Meshes`: 載入的網格物件列表
   - `Paths`: 對應的檔案路徑列表

## gh_component：urdf_loader.py

### 功能
解析 URDF 檔案，將電池01載入的網格與 URDF 中的 link 配對，輸出變換後的機器人模型。

### 在 Grasshopper 中使用

1. 新增一個 **GhPython** 元件
2. 設定輸入參數：
   - `URDFPath` (str): URDF 檔案路徑（例如 `.../walker_arm.urdf`）
   - `Meshes` (list): 來自電池01的 Meshes 輸出
   - `MeshPaths` (list): 來自電池01的 Paths 輸出
   - `J` (list): 關節值列表 [J1, J2, J3, J4, J5, J6]
   - `Deg` (bool): True 表示 J 為角度，False 為弧度（預設 True）
   - `Reload` (bool): 強制重新載入，清除快取（預設 False）

3. 設定輸出參數：
   - `G`: 變換後的機器人網格列表
   - `Names`: 對應的 link 名稱
   - `JointOrder`: 關節順序（J 對應的關節名稱）


## 完整 Grasshopper 連接範例

```
┌─────────────┐
│ Panel       │ DirPath = "/Users/.../urdf/walker_arm"
└──────┬──────┘
       │
       v
┌─────────────────────────────────────┐
│ GhPython (電池01: mesh_loader)      │
│ Inputs: DirPath, Reload             │
│ Outputs: Meshes, Paths              │
└──────┬──────────────┬───────────────┘
       │              │
       │ Meshes       │ Paths
       │              │
       v              v
┌─────────────────────────────────────┐
│ GhPython (電池02: urdf_loader)      │
│ Inputs: URDFPath, Meshes, Paths, J, │
│         Deg, Reload                 │
│ Outputs: G, Names, JointOrder       │
└──────┬──────────────────────────────┘
       │
       v G (機器人模型)
┌─────────────┐
│ Custom      │ 預覽或進一步處理
│ Preview     │
└─────────────┘
```

