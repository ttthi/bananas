import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import sys
from dataclasses import dataclass

@dataclass
class PlanePatchesParams:
    normal_variance_threshold_deg: float = 30
    coplanarity_deg: float = 45
    outlier_ratio: float = 0.5
    min_plane_edge_length: float = 0
    min_num_points: int = 10
    knn: int = 15

def load_point_cloud(file_path):
    """Load the point cloud from a PLY file."""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def remove_radius_outliers(pcd, nb_points=10, radius=0.05, vis=False):
    """Remove radius outliers from the point cloud."""
    cl, ind = pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    inlier_pcd = pcd.select_by_index(ind)
    if vis:
        o3d.visualization.draw_geometries([inlier_pcd], window_name="Radius Inlier Point Cloud", width=800, height=600)
    return inlier_pcd

def remove_statistical_outliers(pcd, nb_neighbors=10, std_ratio=2.0, vis=False):
    """Remove statistical outliers from the point cloud."""
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    inlier_pcd = pcd.select_by_index(ind)
    if vis:
        o3d.visualization.draw_geometries([inlier_pcd], window_name="Statistical Inlier Point Cloud", width=800, height=600)
    return inlier_pcd

def ensure_normals(pcd, knn=30):
    """Ensure point cloud has normal vectors."""
    if not pcd.has_normals():
        print("Computing point cloud normal vectors...")
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamKNN(knn=knn)
        )
        pcd.orient_normals_consistent_tangent_plane(k=knn)
    return pcd

def detect_plane_patches(pcd, vis=False, params: PlanePatchesParams = None):
    """Detect planar patches (floor) from the point cloud."""
    planes = []
    plane_models = []
    min_inliers = 100
    
    # Create colormap
    colors = plt.get_cmap("tab20")(np.linspace(0, 1, 20))[:, :3]
    color_idx = 0

    while len(pcd.points) > min_inliers:
        # Segment the largest plane in the current point cloud
        print(f"Segmenting plane with {len(pcd.points)} points...")
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )
        print("len of inliers", len(inliers))
        if len(inliers) < min_inliers:
            break  # Stop if the detected plane doesn't have enough inliers

        # Extract the plane and paint it with a unique color
        plane = pcd.select_by_index(inliers)
        plane.paint_uniform_color(colors[color_idx % len(colors)])
        planes.append(plane)
        plane_models.append(plane_model)
        
        # Increment color index
        color_idx += 1
        
        # Remove the plane inliers from the point cloud
        pcd = pcd.select_by_index(inliers, invert=True)

    # Paint remaining points gray
    if len(pcd.points) > 0:
        pcd.paint_uniform_color([0.8, 0.8, 0.8])

    # Visualize the segmented planes and the remaining points (if any)
    if vis:
        # Create visualizer for better control
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Detected Planes", width=800, height=600)
        
        # Add all geometries
        for plane in planes:
            vis.add_geometry(plane)
        if len(pcd.points) > 0:
            vis.add_geometry(pcd)
            
        # Set render options
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])  # black background
        opt.point_size = 2.0  # larger points
        
        vis.run()
        vis.destroy_window()

    return planes, plane_models, pcd

def remove_floor_points(pcd, distance_threshold=0.01, vis=False, verbose=False):
    """Remove points that are close to the detected floor plane."""
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                         ransac_n=3,
                                         num_iterations=1000)
    [a, b, c, d] = plane_model
    if verbose:
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    if vis:
        o3d.visualization.draw_geometries([outlier_cloud, inlier_cloud], window_name="Point Cloud Without Floor", width=800, height=600)

    # Fit bounding box to the inlier cloud
    obb = inlier_cloud.get_oriented_bounding_box()

    return outlier_cloud, obb

def cluster_boxes(pcd, eps=0.005, min_points=10, vis=False):
    """Cluster the remaining points into box clusters."""
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=False))

    if vis:
        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw_geometries([pcd], window_name="Clustered Point Cloud", width=800, height=600)

    return labels

def compute_bounding_boxes(pcd, labels, floor_normal=None, floor_center=None, vis=False):
    """Compute and adjust oriented bounding boxes for each cluster."""
    max_label = labels.max()
    bounding_boxes_temp = []
    bounding_boxes = []
    
    for cluster_id in range(max_label + 1):
        cluster_indices = np.where(labels == cluster_id)[0]
        cluster_pcd = pcd.select_by_index(cluster_indices)
        obb = cluster_pcd.get_minimal_oriented_bounding_box()
        obb.color = (1, 0, 0)
        bounding_boxes_temp.append(obb)

    # Find the max volume
    max_volume = max([np.prod(obb.extent) for obb in bounding_boxes_temp])
    for obb in bounding_boxes_temp:
        if np.prod(obb.extent) > 0.1 * max_volume:
            bounding_boxes.append(obb)

    
    if vis: # visualize the point cloud and the bounding boxes
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Locate Bounding Boxes", width=800, height=600)
        vis.add_geometry(pcd)
        for obb in bounding_boxes:
            vis.add_geometry(obb)
        vis.run()
        vis.destroy_window()
    return bounding_boxes

def process_point_cloud(pcd, vis=False, verbose=False):
    """
    Process the input pcd with modified order:
    1. Remove outliers
    2. Remove floor
    3. Cluster points
    4. Detect plane patches for each cluster
    """
    # Visualize the input point cloud
    if vis:
        o3d.visualization.draw_geometries([pcd], window_name="Input Point Cloud", width=800, height=600)

    # Load and clean up the point cloud
    aabb = pcd.get_axis_aligned_bounding_box()
    extent = aabb.get_extent()
    scale = (extent[0] + extent[1] + extent[2]) / 3

    n_points = len(pcd.points)
    if verbose:
        print(f"Number of points: {n_points}")
        print(f"scale: {scale}")
        print (f"Radius threshold: {scale/100}")

    # visualize a sphere with the scale/100 at the floor center
    if vis:
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=scale/100)
        sphere.compute_vertex_normals()
        sphere.paint_uniform_color([0.1, 0.1, 0.7])
        sphere.translate(pcd.get_center())
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Floor Sphere", width=800, height=600)
        vis.add_geometry(pcd)
        vis.add_geometry(sphere)
        vis.run()
        vis.destroy_window()

    pcd = remove_radius_outliers(pcd, nb_points=n_points//5000, radius=scale/100, vis=vis)
    
    # Remove floor points to isolate box clusters
    pcd_without_floor, floor_obb = remove_floor_points(pcd, distance_threshold=scale/100, vis=vis, verbose=verbose)
    
    # First cluster the points
    labels = cluster_boxes(pcd_without_floor, eps=scale/100, min_points=n_points//10000, vis=vis)

    # Compute bounding boxes for each cluster
    bounding_boxes = compute_bounding_boxes(
        pcd_without_floor, labels, vis=vis
    )
    
    return {
        "pcd": pcd,
        "pcd_without_floor": pcd_without_floor,
        "cluster_labels": labels,
        "bounding_boxes": bounding_boxes
    }

def main(*args, **kwargs):
    import argparse
    parser = argparse.ArgumentParser(description='View PLY point cloud file.')
    parser.add_argument('--ply_file', '-f', type=str, help='Path to the PLY file', required=False)
    parser.add_argument('--visualize', '-v', action='store_true', help='Enable visualization')
    parser.add_argument('--verbose', '-vb', action='store_true', help='Enable verbose output')
    parser.add_argument('ply_file_pos', nargs='?', type=str, help='Path to the PLY file (positional)', default=None)
    
    # Parse arguments
    args = parser.parse_args()
    
    # Priority: kwargs > named argument > positional argument
    if 'ply_file' in kwargs:
        ply_file = kwargs['ply_file']
    elif args.ply_file:
        ply_file = args.ply_file
    elif args.ply_file_pos:
        ply_file = args.ply_file_pos
    else:
        parser.print_help()
        sys.exit(1)

    # Check for visualization and verbose flags in kwargs or args
    vis = kwargs.get('visualize', args.visualize)
    verbose = kwargs.get('verbose', args.verbose)

    # Load the point cloud from file
    pcd = o3d.io.read_point_cloud(ply_file)
    result = process_point_cloud(pcd, vis=vis, verbose=verbose)
    
    # Only visualize final result if visualization is enabled
    if vis:
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="Processed Point Cloud", width=800, height=600)
        vis.add_geometry(result["pcd"])
        for obb in result["bounding_boxes"]:
            vis.add_geometry(obb)
        vis.run()
        vis.destroy_window()

if __name__ == "__main__":
    main()