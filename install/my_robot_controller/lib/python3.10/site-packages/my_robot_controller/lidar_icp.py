#!/usr/bin/env python3

import numpy as np
import open3d as o3d

def compute_icp(source_points, target_points, init_transform=np.eye(4), threshold=0.02):
    """
    Menghitung posisi dan orientasi menggunakan ICP.
    Args:
        source_points: Numpy array dari point cloud terbaru.
        target_points: Numpy array dari point cloud referensi.
        init_transform: Transformasi awal (default: identity matrix).
        threshold: Batas jarak maksimum untuk mencocokkan titik (default: 0.02).

    Returns:
        transform: Matriks transformasi (4x4) yang mencakup rotasi dan translasi.
        information: Informasi tambahan hasil ICP.
    """
    # Buat point cloud dari array numpy
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_points)

    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(target_points)

    # Jalankan algoritma ICP
    result = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init_transform,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )

    return result.transformation, result.information

# Contoh data Lidar
source_points = np.random.rand(100, 3)  # Simulasi data terbaru
target_points = source_points + np.array([0.1, 0.1, 0])  # Simulasi data referensi

# Inisialisasi transformasi awal
init_transform = np.eye(4)

# Jalankan ICP
transformation, information = compute_icp(source_points, target_points, init_transform)

# Ekstrak posisi dan orientasi dari transformasi
position = transformation[:3, 3]
rotation_matrix = transformation[:3, :3]

# Hitung orientasi (yaw) dari rotasi
yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

print("Posisi (x, y, z):", position)
print("Orientasi (yaw):", np.degrees(yaw))
