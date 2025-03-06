import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def load_point_cloud(file_path):
    """Load the point cloud from a PLY file."""
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def remove_outliers(pcd, nb_neighbors=10, std_ratio=2.0):
    """Remove statistical outliers from the point cloud."""
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return pcd.select_by_index(ind)

def detect_floor_plane(pcd, **kwargs):
    """Detect planar patches (floor) from the point cloud."""
    oboxes = pcd.detect_planar_patches(
        normal_variance_threshold_deg=kwargs.get("normal_variance_threshold_deg", 80),
        coplanarity_deg=kwargs.get("coplanarity_deg", 45),
        outlier_ratio=kwargs.get("outlier_ratio", 0.89),
        min_plane_edge_length=kwargs.get("min_plane_edge_length", 0),
        min_num_points=kwargs.get("min_num_points", 0),
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=kwargs.get("knn", 100))
    )
    # Choose the largest plane (by bounding-box area)
    plane_areas = []
    for i, obox in enumerate(oboxes):
        ex, ey, ez = obox.extent
        sorted_extent = sorted([ex, ey, ez], reverse=True)
        plane_area = sorted_extent[0] * sorted_extent[1]
        plane_areas.append((i, plane_area))
    plane_areas.sort(key=lambda x: x[1], reverse=True)
    if not plane_areas:
        return None, None
    largest_plane_idx, _ = plane_areas[0]
    return oboxes[largest_plane_idx], oboxes

def remove_floor_points(pcd, floor_obox, floor_normal):
    """Remove points belonging to the detected floor plane."""
    points = np.asarray(pcd.points) - floor_obox.center
    points_local = np.dot(points, floor_obox.R)
    half_extent = floor_obox.extent * 0.75
    mask = np.all(np.abs(points_local) <= half_extent, axis=1)
    inlier_indices = np.where(mask)[0]
    
    all_indices = set(range(len(pcd.points)))
    keep_indices = list(all_indices - set(inlier_indices))
    return pcd.select_by_index(keep_indices)

def cluster_boxes(pcd, eps=0.005, min_points=10):
    """Cluster the remaining points into box clusters."""
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    return labels

def compute_bounding_boxes(pcd, labels, minimum_box_volume=1e-5, floor_normal=None, floor_center=None):
    """Compute and adjust oriented bounding boxes for each cluster."""
    max_label = labels.max()
    bounding_boxes = []
    bounding_boxes_processed = []
    
    for cluster_id in range(max_label + 1):
        cluster_indices = np.where(labels == cluster_id)[0]
        cluster_pcd = pcd.select_by_index(cluster_indices)
        obb = cluster_pcd.get_minimal_oriented_bounding_box()
        obb.color = (1, 0, 0)
        volume = np.prod(obb.extent)
        if volume < minimum_box_volume:
            continue
        bounding_boxes.append(obb)
        
        # Adjust the box to align with the floor if a floor normal is provided
        obb_processed = cluster_pcd.get_minimal_oriented_bounding_box()
        if floor_normal is not None and floor_center is not None:
            corners = np.asarray(obb_processed.get_box_points())
            heights = corners.dot(floor_normal)
            h_min = heights.min()
            floor_level = np.dot(floor_normal, floor_center)
            delta = h_min - floor_level
            if delta > 0:
                alignments = np.abs(obb_processed.R.T.dot(floor_normal))
                idx = np.argmax(alignments)
                new_extent = np.array(obb_processed.extent)
                new_extent[idx] += delta
                obb_processed.center = obb_processed.center - floor_normal * (delta / 2)
                obb_processed.extent = new_extent
        # Adjusted boxes are colored in purple
        obb_processed.color = (1, 0, 1)
        bounding_boxes_processed.append(obb_processed)
    
    return bounding_boxes, bounding_boxes_processed

def process_point_cloud(pcd):
    """
    Process the input pcd and return a dictionary with:
      - 'visible_pcd': Processed point cloud after outlier removal.
      - 'floor_obox': Detected floor plane (oriented bounding box).
      - 'bounding_boxes': List of raw bounding boxes for the clusters.
      - 'bounding_boxes_processed': List of adjusted bounding boxes.
    """
    # Load and clean up the point cloud
    pcd = remove_outliers(pcd)
    
    # Detect the floor plane
    floor_obox, all_planes = detect_floor_plane(pcd)
    if floor_obox is None:
        raise ValueError("No floor plane detected.")
    floor_normal = floor_obox.R[:, 2]
    
    # Remove floor points to isolate box clusters
    pcd_without_floor = remove_floor_points(pcd, floor_obox, floor_normal)
    
    # Cluster the remaining points
    labels = cluster_boxes(pcd_without_floor)
    
    # Compute bounding boxes for each cluster
    bounding_boxes, bounding_boxes_processed = compute_bounding_boxes(
        pcd_without_floor, labels, floor_normal=floor_normal, floor_center=floor_obox.center
    )
    
    return {
        "visible_pcd": pcd,
        "floor_obox": floor_obox,
        "floor_normal": floor_normal,
        "pcd_without_floor": pcd_without_floor,
        "cluster_labels": labels,
        "bounding_boxes": bounding_boxes,
        "bounding_boxes_processed": bounding_boxes_processed,
    }

if __name__ == "__main__":

    # Load the point cloud from a PLY file
    pcd = load_point_cloud("./data/14022025-2box.ply")
    result = process_point_cloud(pcd)
    
    # For instance, visualize the adjusted bounding boxes along with the original point cloud.
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="Processed Point Cloud")
    vis.add_geometry(result["visible_pcd"])
    for obb in result["bounding_boxes_processed"]:
        vis.add_geometry(obb)
    vis.run()
    vis.destroy_window()
