import numpy as np
import open3d as o3d
import cv2

class Floor:
    """
    Object which stores helper functions for handling the floor of the room
    """
    def __init__(self, calibration_points, verbose=False):
        self.calibration_points = calibration_points
        self.floor_plane = self.calculate_floor_plane(verbose=verbose)
        self.t, self.R = self.get_plane_transforms(self.floor_plane)

        # Persistent min/max XY bounds for visualization
        self.min_xy = None
        self.max_xy = None


    def calculate_floor_plane(self, verbose=False):
        p1 = np.array(self.calibration_points[0])
        p2 = np.array(self.calibration_points[1])
        p3 = np.array(self.calibration_points[2])

        #Vectors from points (treat p1 as origin)
        v1 = p2 - p1
        v2 = p3 - p1

        #Find normal vector
        norm = np.cross(v1, v2)

        #Get plane components
        a, b, c = norm
        d = - (a * p1[0] + b * p1[1] + c * p1[2])

        self.floor_plane = np.array([a,b,c,d])

        if verbose:
            print(f"Floor plane equation: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")

        return self.floor_plane
    
    def get_plane_transforms(self, plane):
        """
        Calulates the rotation and translation to reorient point cloud with the
        parameter plane as the XY plane

        plane: (4,) tuple, a b c d of plane coeffs
        
        returns
            plane_point - translation vector
            R - rotation matrix
        """
        a, b, c, d = plane
        plane_point = self.calibration_points[0]
        plane_normal = np.array([a,b,c])


        # Normalize the plane normal
        n = plane_normal / np.linalg.norm(plane_normal)
        
        # Z-axis unit vector (target normal)
        z_axis = np.array([0, 0, 1.0])
        
        # Compute rotation axis (cross product) and angle
        axis = np.cross(n, z_axis)
        axis_norm = np.linalg.norm(axis)
        
        if axis_norm < 1e-8:  
            # Normal is already aligned with Z-axis
            R = np.eye(3)
        else:
            axis /= axis_norm
            angle = np.arccos(np.clip(np.dot(n, z_axis), -1.0, 1.0))
            
            # Rodrigues' rotation formula
            K = np.array([[0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]])
            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
        
        self.R = R
        self.t = plane_point
        return plane_point, R
    
    def transform_to_floor_frame(self, points):
        """
        Apply translation + rotation to move points into floor-aligned frame.
        """
        pts = points - self.t  # translate to plane origin
        pts = pts @ self.R.T   # rotate so floor normal aligns with +Z
        return pts
    
    def find_floor(self, room_pc, threshold):
        """
        Splits the point cloud into floor points and non-floor points.
        Assumes room_pc is already in the floor-aligned frame (Z ≈ 0 for floor).

        Returns
        -------
        floor_points : points within threshold of z=0
        non_floor_points : everything else (obstacles, walls, human)
        """

        if len(room_pc) == 0:
            return (
                np.empty((0,3), dtype=np.float32),
                np.empty((0,3), dtype=np.float32)
            )

        z = room_pc[:, 2]
        mask_floor = np.abs(z) <= threshold

        floor_points = room_pc[mask_floor]
        non_floor_points = room_pc[~mask_floor]
        return floor_points, non_floor_points

    
    def visualize_floor_map(self, floor_points, non_floor_points,
                        img_size=800, scale=0.1):
        """
        Creates a 2D top-down RGB visualization:
            - Floor points appear BLUE
            - Non-floor points (objects/humans) appear RED

        Uses persistent min/max XY to avoid jitter.
        """

        # If no floor detected, return blank image
        if len(floor_points) == 0:
            return np.zeros((img_size, img_size, 3), dtype=np.uint8)

        # Extract XY coordinates of floor points
        pts_xy = floor_points[:, :2]

        # Init persistent XY bounds
        if self.min_xy is None or self.max_xy is None:
            self.min_xy = pts_xy.min(axis=0)
            self.max_xy = pts_xy.max(axis=0)
        else:
            current_min = pts_xy.min(axis=0)
            current_max = pts_xy.max(axis=0)
            self.min_xy = np.minimum(self.min_xy, current_min)
            self.max_xy = np.maximum(self.max_xy, current_max)

        # Normalize with persistent min
        pts_norm = pts_xy - self.min_xy
        floor_size = self.max_xy - self.min_xy
        size_pixels = (floor_size * scale).astype(int)

        canvas_size = int(max(size_pixels) + 20)
        canvas_size = max(canvas_size, img_size)

        # Create RGB canvas
        img = np.zeros((canvas_size, canvas_size, 3), dtype=np.uint8)

        # -----------------------------------------------------
        # Draw Floor Points (BLUE)
        # -----------------------------------------------------
        pts_floor_pix = (pts_norm * scale).astype(int)

        for x, y in pts_floor_pix:
            if 0 <= x < canvas_size and 0 <= y < canvas_size:
                img[y, x] = (255, 0, 0)  # Blue (B,G,R) → (255,0,0)

        # -----------------------------------------------------
        # Draw Non-Floor Points (RED)
        # -----------------------------------------------------
        if len(non_floor_points) > 0:
            pts_obs = non_floor_points[:, :2] - self.min_xy
            pts_obs_pix = (pts_obs * scale).astype(int)

            for x, y in pts_obs_pix:
                if 0 <= x < canvas_size and 0 <= y < canvas_size:
                    img[y, x] = (0, 0, 255)  # Red (B,G,R) → (0,0,255)

        # Resize final output
        img = cv2.resize(img, (img_size, img_size), interpolation=cv2.INTER_NEAREST)
        return img

    
    def visualize_height_map(self, points_xyz, img_size=800, scale=0.1,
                         z_min=None, z_max=None):
        """
        Visualizes ALL point cloud points as a heatmap based on their height (Z)
        after being transformed into the floor reference frame.

        - X,Y determine pixel location (top-down)
        - Z determines color (heat value)
        - Uses persistent min/max XY bounds to avoid jitter (same as visualize_floor_map)

        Parameters:
            points_xyz : (N,3) array of 3D points in floor frame
            img_size   : final output image size in pixels
            scale      : XY-to-pixel scaling factor
            z_min/z_max: optional fixed height range for the colormap

        Returns:
            heatmap_img : (img_size,img_size,3) RGB heatmap (uint8)
        """

        if len(points_xyz) == 0:
            return np.zeros((img_size, img_size, 3), dtype=np.uint8)

        pts_xy = points_xyz[:, :2]
        pts_z  = points_xyz[:, 2]

        # Normalize XY into pixel space
        pts_norm = pts_xy - self.min_xy
        floor_size = (self.max_xy - self.min_xy)
        pixel_size_vec = floor_size * scale
        canvas_size = int(max(pixel_size_vec) + 20)
        canvas_size = max(canvas_size, img_size)

        # Blank heatmap canvas (float for colormap)
        heatmap = np.zeros((canvas_size, canvas_size), dtype=np.float32)

        # ----- Normalize Z (height) -----
        if z_min is None: z_min = pts_z.min()
        if z_max is None: z_max = pts_z.max()
        if z_max - z_min < 1e-6:
            z_max = z_min + 1e-6

        z_norm = (pts_z - z_min) / (z_max - z_min)
        z_norm = np.clip(z_norm, 0.0, 1.0)

        # Convert XY → pixel
        pts_pix = (pts_norm * scale).astype(int)

        # Plot into heatmap (take max height if multiple points collide)
        for (x, y), zval in zip(pts_pix, z_norm):
            if 0 <= x < canvas_size and 0 <= y < canvas_size:
                heatmap[y, x] = max(heatmap[y, x], zval)

        # Apply a colormap (JET / TURBO / MAGMA / etc.)
        heatmap_color = cv2.applyColorMap(
            (heatmap * 255).astype(np.uint8),
            cv2.COLORMAP_JET
        )

        # Resize to consistent output
        heatmap_color = cv2.resize(heatmap_color, (img_size, img_size),
                                interpolation=cv2.INTER_NEAREST)

        return heatmap_color