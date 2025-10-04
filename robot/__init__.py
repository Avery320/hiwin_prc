"""
robot - Grasshopper URDF 機器人載入器

這個模組提供兩個主要功能：
1. mesh_loader - 載入網格檔案（電池01）
2. urdf_loader - 解析 URDF 並計算運動學（電池02）

作者：Avery Tsai
版本：2.0
日期：2025-10-05
"""

__version__ = '2.0'
__author__ = 'Avery Tsai'

# 匯入主要 API
from .mesh_loader import load as load_meshes, clear_cache as clear_mesh_cache
from .urdf_loader import load as load_urdf, clear_cache as clear_urdf_cache

__all__ = [
    'load_meshes',
    'load_urdf',
    'clear_mesh_cache',
    'clear_urdf_cache',
]

