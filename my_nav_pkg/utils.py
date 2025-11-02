import yaml
import numpy as np
import cv2
import os

def load_occupancy_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)

    # 👇 make image path absolute relative to YAML
    yaml_dir = os.path.dirname(os.path.abspath(yaml_path))
    pgm_path = info['image']
    if not os.path.isabs(pgm_path):
        pgm_path = os.path.join(yaml_dir, pgm_path)

    origin = info['origin']  # [x, y, yaw]
    resolution = info['resolution']
    negate = info.get('negate', 0)
    occupied_thresh = info.get('occupied_thresh', 0.65)
    free_thresh = info.get('free_thresh', 0.196)

    img = cv2.imread(pgm_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError(f"Cannot read map image {pgm_path}")

    # 0..1
    occ = (255 - img) / 255.0
    occ_grid = np.zeros_like(occ, dtype=np.uint8)
    occ_grid[occ > occupied_thresh] = 100
    occ_grid[occ < free_thresh] = 0
    occ_grid[(occ <= occupied_thresh) & (occ >= free_thresh)] = 100  # unknown -> occupied

    return occ_grid, origin, resolution

def inflate_costmap(occ_grid, robot_radius_m, resolution_m, blur_kernel=7):
    """
    occ_grid: 0 free, 100 occupied
    We will:
      1. distance transform
      2. assign higher cost to cells close to obstacles
      3. gaussian blur to make it look 'soft'
    """
    # 1 -> free, 0 -> obstacle for distanceTransform
    free_mask = (occ_grid == 0).astype(np.uint8)
    dist = cv2.distanceTransform(free_mask, cv2.DIST_L2, 3)
    # meters
    dist_m = dist * resolution_m

    # cells within robot radius => occupied
    inflated = occ_grid.copy()
    inflated[dist_m < robot_radius_m] = 100

    # build soft cost: closer -> higher
    max_influence = robot_radius_m * 2.5
    soft_cost = np.clip((max_influence - dist_m) / max_influence, 0, 1) * 90
    # merge: take max of existing inflated and soft cost
    soft_cost = soft_cost.astype(np.uint8)
    final = np.maximum(inflated, soft_cost)

    # blur to look like "blurred image" costmap
    final = cv2.GaussianBlur(final, (blur_kernel, blur_kernel), 0)

    return final
