from vision import detection, floor, watcher, path, robot
import cv2
import numpy as np
import time
import json
import os

# -------------------- Calibration Manager --------------------

class CalibrationManager:
    def __init__(self, cam_1, cam_2, always_default=True, default_room="2-12-26_calibration.json"):
        self.cam_1 = cam_1
        self.cam_2 = cam_2

        self.calib_points_cam1 = []
        self.calib_points_cam2 = []
        self.calib_mode = None
        self.calib_done = False

        self.calib_R = None
        self.calib_t = None

        self.floor_threshold = 800
        self.cluster_epsilon = 1000
        self.cluster_min_samples = 150

        self.floor_done = False

        if always_default:
            self.load_room_file(default_room)
        else:
            resp = input("Enter 'y' for new calibration, or filename to load: ")
            if resp != 'y':
                self.load_room_file(resp)
            else:
                self.cam_1.display = True
                self.cam_2.display = True

    # ---------- File I/O ----------

    def load_room_file(self, room_file):
        with open(f"rooms/{room_file}", 'r') as f:
            room_data = json.load(f)

        self.calib_points_cam1 = np.array(room_data["cam1"]["3D_points"])
        self.calib_points_cam2 = np.array(room_data["cam2"]["3D_points"])
        self.calib_R = np.array(room_data["rotation"])
        self.calib_t = np.array(room_data["translation"])

        self.calib_done = True

        print("===== CAMERA CALIBRATION COMPLETE =====")
        print("Rotation:\n", self.calib_R)
        print("Translation:\n", self.calib_t)

    def save_or_update_calibration(self):
        room_name = input("Enter room name: ")
        cam1_location = input("Camera 1 location description: ")
        cam2_location = input("Camera 2 location description: ")

        room_data = {
            "name": room_name,
            "cam1": {
                "location": cam1_location,
                "3D_points": np.asarray(self.calib_points_cam1).tolist()
            },
            "cam2": {
                "location": cam2_location,
                "3D_points": np.asarray(self.calib_points_cam2).tolist()
            },
            "rotation": np.asarray(self.calib_R).tolist(),
            "translation": np.asarray(self.calib_t).tolist()
        }

        os.makedirs("rooms", exist_ok=True)
        save_path = f"rooms/{room_name}.json"

        if os.path.exists(save_path):
            with open(save_path, 'r') as f:
                existing = json.load(f)
            existing.update(room_data)
            room_data = existing

        with open(save_path, 'w') as f:
            json.dump(room_data, f, indent=4)

        print(f"Calibration saved to {save_path}")

    # ---------- Calibration Math ----------

    def compute_camera_transform(self):
        if len(self.calib_points_cam1) != 3 or len(self.calib_points_cam2) != 3:
            print("Need exactly 3 points from each camera")
            return False

        P = np.array(self.calib_points_cam1, dtype=np.float64)
        Q = np.array(self.calib_points_cam2, dtype=np.float64)

        Pc = P.mean(axis=0)
        Qc = Q.mean(axis=0)

        H = (Q - Qc).T @ (P - Pc)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T

        t = Pc - R @ Qc

        self.calib_R = R
        self.calib_t = t

        print("===== CAMERA CALIBRATION COMPLETE =====")
        print("Rotation:\n", R)
        print("Translation:\n", t)

        return True

    # ---------- Mouse Handling ----------

    def click_to_3d_point(self, x, y, depth, cam):
        z = depth[y, x]
        if z == 0:
            return None
        X = (x - cam.cx) * z / cam.fx
        Y = (y - cam.cy) * z / cam.fy
        return np.array([X, Y, z], dtype=np.float32)

    def calibration_mouse_callback(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.calib_mode == "cam1":
            pt = self.click_to_3d_point(x, y, self.cam_1.depth, self.cam_1)
            if pt is not None:
                self.calib_points_cam1.append(pt)
                print("[CAM1]", pt)

        elif self.calib_mode == "cam2":
            pt = self.click_to_3d_point(x, y, self.cam_2.depth, self.cam_2)
            if pt is not None:
                self.calib_points_cam2.append(pt)
                print("[CAM2]", pt)

    # ---------- State Machine ----------

    def start_calibration(self):
        self.calib_mode = "cam1"
        self.calib_points_cam1.clear()
        self.calib_points_cam2.clear()
        self.calib_done = False
        self.floor_done = False

        print("CLICK 3 POINTS IN CAM1 WINDOW")

        self.cam_2.stop_stream()
        self.cam_2.display = False
        self.cam_2.destroy_windows()

    def update_calibration(self):
        if self.calib_mode == "cam1" and len(self.calib_points_cam1) == 3:
            self.calib_mode = "cam2"
            print("CLICK 3 CORRESPONDING POINTS IN CAM2")

            self.cam_1.stop_stream()
            self.cam_1.display = False
            self.cam_1.destroy_windows()

            self.cam_2.start_stream()
            self.cam_2.display = True
            self.cam_2.create_windows()

        elif self.calib_mode == "cam2" and len(self.calib_points_cam2) == 3:
            if self.compute_camera_transform():
                self.calib_done = True
                self.save_or_update_calibration()

            self.cam_1.start_stream()
            self.cam_1.display = True
            self.cam_1.create_windows()

            self.cam_2.start_stream()
            self.cam_2.display = True
            self.cam_2.create_windows()

            self.calib_mode = None

    # ---------- Live Parameters ----------

    def update_variables(self, key):
        if key == ord('f'):
            self.floor_threshold = float(input("New floor threshold: "))

        if key == ord('g'):
            self.cluster_epsilon = float(input("New epsilon: "))
            self.cluster_min_samples = int(input("New min_samples: "))

# -------------------- Utility --------------------

def depth_to_point_cloud_subsample(depth, cam, stride=4):
    h, w = depth.shape
    u = np.arange(0, w, stride)
    v = np.arange(0, h, stride)
    uu, vv = np.meshgrid(u, v)

    z = depth[vv, uu].astype(np.float32)
    mask = z > 0

    uu = uu[mask]
    vv = vv[mask]
    z = z[mask]

    X = (uu - cam.cx) * z / cam.fx
    Y = (vv - cam.cy) * z / cam.fy

    return np.column_stack((X, Y, z))

def world_to_pixel(world_xy, min_xy, scale, canvas, img_size):
    """
    Converts world coordinates to final displayed image pixel coordinates.

    world_xy : np.array([x, y]) in world units
    min_xy   : floor min bound (world)
    scale    : world → canvas scale
    canvas   : internal canvas size (before resize)
    img_size : final displayed image size
    """

    # World → canvas
    canvas_xy = (world_xy - min_xy) * scale

    # Canvas → display
    resize_scale = img_size / canvas
    pixel_xy = canvas_xy * resize_scale

    return pixel_xy.astype(int)

clicked_point = None

def cluster_mouse_callback(event, x, y, flags, param):
        global clicked_point

        if event != cv2.EVENT_LBUTTONDOWN:
            return
        
        clicked_point = (x,y)

# ------------------------------------------------------------
# Select and save Roomba cluster
# ------------------------------------------------------------
def prompt_and_save_roomba(clusterer):
    if not clusterer.clusters:
        print("No clusters available")
        return

    print("\nAvailable clusters:")
    for c in clusterer.clusters:
        tid = c["track_id"]
        chamfer = c["chamfer"]
        npts = len(c["points_xyz"])
        print(f"  ID {tid}: chamfer={chamfer:.3f}, points={npts}")

    try:
        tid = int(input("\nEnter cluster ID corresponding to Roomba: "))
    except ValueError:
        print("Invalid input")
        return

    for c in clusterer.clusters:
        if c["track_id"] == tid:
            pts = c["points_xyz"]

            # --- center the model ---
            centroid = pts.mean(axis=0)
            pts_centered = pts - centroid

            np.save("roomba_points.npy", pts_centered)

            print(
                f"\nSaved {len(pts_centered)} points "
                f"to roomba_points.npy"
            )
            return

    print("Cluster ID not found")

# -------------------- Startup --------------------

cam_1 = watcher.Watcher(display=True, index=0)
cam_2 = watcher.Watcher(display=True, index=1)

print("Giving runtime buffer...")
time.sleep(3)

calibration_manager = CalibrationManager(cam_1, cam_2)
clusterer = detection.ClusterDetection(
    eps=calibration_manager.cluster_epsilon,
    min_samples=calibration_manager.cluster_min_samples
)
roombeer = robot.Robot()
floor_obj = None

position_1 = None
pre_cal_time = None

# -------------------- Main Loop --------------------

while True:
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        calibration_manager.start_calibration()

    calibration_manager.update_calibration()
    calibration_manager.update_variables(key)
    
    if cam_1.display:
        cv2.imshow(f'KINECT {cam_1.index} Video Stream', cam_1.video)
        cv2.imshow(f'KINECT {cam_1.index} Depth Stream', cam_1.depth)
        cv2.setMouseCallback(
            f'KINECT {cam_1.index} Video Stream',
            calibration_manager.calibration_mouse_callback
        )

    if cam_2.display:
        cv2.imshow(f'KINECT {cam_2.index} Video Stream', cam_2.video)
        cv2.imshow(f'KINECT {cam_2.index} Depth Stream', cam_2.depth)
        cv2.setMouseCallback(
            f'KINECT {cam_2.index} Video Stream',
            calibration_manager.calibration_mouse_callback
        )

    if calibration_manager.calib_done:
        cloud1 = depth_to_point_cloud_subsample(cam_1.depth, cam_1)
        cloud2_uncalibrated = depth_to_point_cloud_subsample(cam_2.depth, cam_2)
        cloud2 = (cloud2_uncalibrated @ calibration_manager.calib_R.T) + calibration_manager.calib_t
        pc = np.vstack((cloud1, cloud2))

        if not calibration_manager.floor_done:
            floor_obj = floor.Floor(calibration_manager.calib_points_cam1, verbose=False)
            calibration_manager.floor_done = True

        pc_floor = floor_obj.transform_to_floor_frame(pc)
        floor_pts, obstacle_pts = floor_obj.find_floor(
            pc_floor,
            threshold=calibration_manager.floor_threshold
        )

        floor_img = floor_obj.visualize_floor_map(
            floor_pts, obstacle_pts, img_size=400, scale=0.025
        )

        clusterer.eps = calibration_manager.cluster_epsilon
        clusterer.min_samples = calibration_manager.cluster_min_samples
        clusterer.min_xy, clusterer.max_xy = floor_obj.min_xy, floor_obj.max_xy

        clusters, labels = clusterer.detect_clusters(obstacle_pts[:, :2])
        clusterer.update_tracks()
        cluster_img = clusterer.visualize_clusters()

        floor_size = clusterer.max_xy - clusterer.min_xy
        canvas = int(max((floor_size * 0.025)) + 20)
        canvas = max(canvas, 400)

        # roomba_cluster = max(clusterer.tracks.items(), key=lambda k: k[1]["wins"])

        # roombeer.position = world_to_pixel(
        #                     roomba_cluster[1]["centroid"],
        #                     clusterer.min_xy,
        #                     0.025,
        #                     canvas,
        #                     400
        #                     )
        # cv2.circle(cluster_img, roombeer.position.astype(int), 4, (255,255,0), 1)

        if key == ord('p'):
            pre_cal_time = time.time()
            if not position_1:
                position_1 = roombeer.position
            
            roombeer.controller.drive(roombeer.velocity, 0)

        if pre_cal_time and (time.time() - pre_cal_time > 2):
            position_2 = roombeer.position
            roombeer.calibrate_roomba_facing_direction(position_1, position_2)
            roombeer.controller.stop()

        if roombeer.facing_direction is not None:

            cv2.arrowedLine(cluster_img, (roombeer.position).astype(int), 
                            (roombeer.position + roombeer.facing_direction * 35).astype(int), 
                            (255,255,255), 2)

        if (clicked_point and roombeer.facing_direction is not None):
            cv2.circle(cluster_img, clicked_point, 4, (255,255,0), 1)
            roombeer.point_roomba_toward_point(clicked_point)

        cv2.imshow("Clusters", cluster_img)
        cv2.setMouseCallback("Clusters", cluster_mouse_callback)

        cv2.imshow("Floor (Top View)", floor_img)

        if key == ord('s'):
            prompt_and_save_roomba(clusterer)

        if key == ord('b'):
            #Save depth image, individual point clouds
            room_image = {
                "cam_1": {
                    "depth": np.asarray(cam_1.depth).tolist(),
                    "point_cloud": np.asarray(cloud1).tolist()
                },
                "cam_2": {
                    "depth": np.asarray(cam_2.depth).tolist(),
                    "point_cloud": np.asarray(cloud2_uncalibrated).tolist()
                }
            }

            save_path = f"example_viz/depth_pc_capture.json"

            if os.path.exists(save_path):
                with open(save_path, 'r') as f:
                    existing = json.load(f)
                existing.update(room_image)
                room_image = existing

            with open(save_path, 'w') as f:
                json.dump(room_image, f, indent=4)

            print(f"Snapshot saved to {save_path}")

    if key == 27:
        break

# -------------------- Cleanup --------------------

cam_1.close()
cam_2.close()
