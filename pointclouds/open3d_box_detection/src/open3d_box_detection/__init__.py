import argparse
import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(prog="detect-box", description="TODO")
    parser.add_argument("pcd_file")
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(args.pcd_file)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(60))
    planes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=60,
        coplanarity_deg=70,
        outlier_ratio=0.75,
        min_plane_edge_length=0,
        min_num_points=0,
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=50),
    )
    planes.sort(key=lambda plane: -plane.volume())
    clouds = [
        pcd.crop(plane).paint_uniform_color(np.random.rand(3)) for plane in planes
    ]

    o3d.visualization.draw_geometries([*clouds, *planes])
