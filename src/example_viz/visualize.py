import numpy as np
import matplotlib.pyplot as plt
import imageio
import json
import tqdm
import io
from PIL import Image
def save_rotating_pointcloud_gif(points, camera_label, output_path="pointcloud.gif",
                                 n_frames=60, elev=20, color="red"):
    assert points.shape[1] == 3, "Input must be Nx3 array"

    # Center the point cloud but KEEP track of the original origin
    # If the camera was at (0,0,0) in the raw data, it stays (0,0,0)
    # If you want the cloud centered, we calculate the offset:
    center = points.mean(axis=0)
    points_centered = points - center
    origin_centered = np.array([0, 0, 0]) - center
    
    lim = np.max(np.linalg.norm(points_centered, axis=1)) * 1.2 # Extra room for label

    frames = []

    for i in tqdm.tqdm(range(n_frames)):
        angle = 360 * i / n_frames

        fig = plt.figure(figsize=(10, 10), facecolor='black')
        ax = fig.add_subplot(111, projection='3d')
        ax.set_facecolor('black')

        # 1. Plot the main point cloud (Red)
        ax.scatter(points_centered[:, 0], points_centered[:, 1], points_centered[:, 2], 
                   s=1.5, color=color, linewidths=0, alpha=0.6)

        # 2. Plot the Origin/Camera (White, Larger)
        ax.scatter(origin_centered[0], origin_centered[1], origin_centered[2], 
                   s=50, color='white', marker='o', edgecolors='white', zorder=10)

        # 3. Label the Origin
        # We add a small offset so the text doesn't sit directly on the point
        ax.text(origin_centered[0], origin_centered[1], origin_centered[2] + (lim * 0.05), 
                camera_label, color='white', fontsize=12, fontweight='bold', ha='center')

        # Formatting
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(-lim, lim)
        ax.set_axis_off()
        ax.grid(False)
        ax.xaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        ax.yaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))
        ax.zaxis.set_pane_color((0.0, 0.0, 0.0, 0.0))

        ax.view_init(elev=elev, azim=angle)

        # Buffer Capture
        buf = io.BytesIO()
        plt.savefig(buf, format='png', facecolor=fig.get_facecolor(), 
                    edgecolor='none', bbox_inches='tight', pad_inches=0.1, dpi=100)
        buf.seek(0)
        
        frames.append(np.array(Image.open(buf)))
        buf.close()
        plt.close(fig)

    imageio.mimsave(output_path, frames, fps=12, loop=0)
    print(f"Saved high-res GIF with camera origin to {output_path}")
from scipy.spatial.transform import Rotation as R_tool
from scipy.spatial.transform import Slerp

def save_sequenced_calibration_gif(pc1, pc2, R_mat, t_vec, calib1, calib2, 
                                   output_path="sequenced_calib.gif", elev=15):
    # --- 1. Calculate Leveling Rotation (Aligning Calib Plane to Z=0) ---
    v1 = calib1[1] - calib1[0]
    v2 = calib1[2] - calib1[0]
    normal = np.cross(v1, v2)
    normal /= np.linalg.norm(normal)
    
    target_up = np.array([0, 0, 1])
    rot_axis = np.cross(normal, target_up)
    if np.linalg.norm(rot_axis) > 1e-6:
        rot_axis /= np.linalg.norm(rot_axis)
        angle = np.arccos(np.clip(np.dot(normal, target_up), -1.0, 1.0))
        leveling_R = R_tool.from_rotvec(rot_axis * angle).as_matrix()
    else:
        leveling_R = np.eye(3)

    # Pre-level Cam 1 and final Cam 2 for centering
    pc1_lev = (leveling_R @ pc1.T).T
    calib1_lev = (leveling_R @ calib1.T).T
    pc2_final_lev = (leveling_R @ ((R_mat @ pc2.T).T + t_vec).T).T
    
    all_pts_final = np.vstack([pc1_lev, pc2_final_lev])
    global_center = all_pts_final.mean(axis=0)
    global_center[2] = calib1_lev[:, 2].mean() # Set "floor" height

    # --- 2. Phase Definitions ---
    n_rot = 40      # Frames for Rotation
    n_pause = 10    # Frames for pauses
    n_trans = 40    # Frames for Translation
    n_spin = 60     # Frames for the final 360 rotation
    total_frames = n_rot + n_pause + n_trans + n_pause + n_spin
    
    slerp = Slerp([0, 1], R_tool.from_matrix([np.eye(3), R_mat]))
    frames = []

    for i in tqdm.tqdm(range(total_frames)):
        # Determine current transformation state
        pc1_color = "red"
        pc2_color = "blue"
        if i < n_rot:
            # Phase 1: Rotate only
            factor = i / (n_rot - 1)
            curr_R = slerp(factor).as_matrix()
            curr_t = np.array([0, 0, 0])
        elif i < n_rot + n_pause:
            # Phase 2: Pause after rotation
            curr_R = R_mat
            curr_t = np.array([0, 0, 0])
        elif i < n_rot + n_pause + n_trans:
            # Phase 3: Translate only
            factor = (i - (n_rot + n_pause)) / (n_trans - 1)
            curr_R = R_mat
            curr_t = t_vec * factor
        else:
            # Phase 4: Final Pause & 360 Spin
            curr_R = R_mat
            curr_t = t_vec
            pc1_color = "darkviolet"
            pc2_color = "darkviolet"

        # Apply current Cam 2 state
        pc2_now = (curr_R @ pc2.T).T + curr_t
        o2_now = (curr_R @ np.array([0,0,0])) + curr_t
        c2_now = (curr_R @ calib2.T).T + curr_t
        
        # Transform everything to Level/Center space
        p1_plot = pc1_lev - global_center
        p2_plot = (leveling_R @ pc2_now.T).T - global_center
        o1_plot = (leveling_R @ np.array([0,0,0])) - global_center
        o2_plot = (leveling_R @ o2_now) - global_center
        c1_plot = calib1_lev - global_center
        c2_plot = (leveling_R @ c2_now.T).T - global_center

        # --- 3. Plotting ---
        fig = plt.figure(figsize=(10, 10), facecolor='black')
        ax = fig.add_subplot(111, projection='3d', facecolor='black')
        
        ax.scatter(p1_plot[:, 0], p1_plot[:, 1], p1_plot[:, 2], s=1.5, color=pc1_color, alpha=0.3)
        ax.scatter(p2_plot[:, 0], p2_plot[:, 1], p2_plot[:, 2], s=1.5, color=pc2_color, alpha=0.3)

        # Origins & Calib markers
        ax.scatter(*o1_plot, s=120, color='white', edgecolors='red', linewidth=2)
        ax.text(*o1_plot, "Camera 1", color='white', fontsize=12, fontweight='bold', ha='center')
        ax.scatter(*o2_plot, s=120, color='white', edgecolors='blue', linewidth=2)
        ax.text(*o2_plot, "Camera 2", color='white', fontsize=12, fontweight='bold', ha='center')
        ax.scatter(c1_plot[:, 0], c1_plot[:, 1], c1_plot[:, 2], s=250, color='lime', marker='X')
        ax.scatter(c2_plot[:, 0], c2_plot[:, 1], c2_plot[:, 2], s=250, color='lime', marker='o', alpha=0.6)

        # View Control
        lim = np.max(np.linalg.norm(all_pts_final - global_center, axis=1)) * 1.1
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(0, lim*1.5)
        ax.set_axis_off()
        
        # Smooth camera rotation: Static during calib, then 360 spin at the end
        if i < (total_frames - n_spin):
            azim = 30
        else:
            # Complete 360 degree spin over the last n_spin frames
            spin_factor = (i - (total_frames - n_spin)) / (n_spin - 1)
            azim = 30 + (spin_factor * 360)
        
        ax.view_init(elev=elev, azim=azim)

        # Output Capture
        buf = io.BytesIO()
        plt.savefig(buf, format='png', facecolor='black', bbox_inches='tight', pad_inches=0, dpi=100)
        buf.seek(0)
        frames.append(np.array(Image.open(buf)))
        plt.close(fig)

    imageio.mimsave(output_path, frames, fps=20, loop=0)
    print(f"Calibration sequence saved to {output_path}")

def save_heatmap(array, output_path="heatmap.png", cmap='inferno'):
    """
    Saves a (H, W, 1) or (H, W) numpy array as a heatmap PNG.
    """
    # Squeeze (480, 640, 1) -> (480, 640)
    data = np.squeeze(array)
    
    # Save directly using the specified colormap
    plt.imsave(output_path, data, cmap=cmap)
    print(f"Heatmap saved to {output_path}")

with open("depth_pc_capture.json", 'r') as f:
    capture_data = json.load(f)

with open("../rooms/2-12-26_calibration.json") as f:
    room_data = json.load(f)

calib_points_cam1 = np.array(room_data["cam1"]["3D_points"])
calib_points_cam2 = np.array(room_data["cam2"]["3D_points"])
calib_R = np.array(room_data["rotation"])
calib_t = np.array(room_data["translation"])

cam_1_pc = np.array(capture_data["cam_1"]["point_cloud"])
cam_1_depth = np.array(capture_data["cam_1"]["depth"])
cam_2_pc = np.array(capture_data["cam_2"]["point_cloud"])
cam_2_depth = np.array(capture_data["cam_2"]["depth"])

save_heatmap(cam_1_depth, output_path="cam_1_depth_heatmap.png")
save_heatmap(cam_2_depth, output_path="cam_2_depth_heatmap.png")

# save_rotating_pointcloud_gif(cam_1_pc, "Camera 1", output_path="cam_1_pointcloud.gif", n_frames=36)
# save_rotating_pointcloud_gif(cam_2_pc, "Camera_2", color="blue", output_path="cam_2_pointcloud.gif", n_frames=36)

# save_sequenced_calibration_gif(
#     cam_1_pc, cam_2_pc, calib_R, calib_t, 
#     calib_points_cam1, calib_points_cam2
# )

# from matplotlib.widgets import CheckButtons
# def visualize_with_toggles(pc1, pc2, calib_R, calib_t, calib_points1):
#     # --- 1. Transformation (Matching your Live Stack) ---
#     pc2_transformed = (pc2 @ calib_R.T) + calib_t
#     origin2_transformed = (np.array([[0, 0, 0]]) @ calib_R.T) + calib_t
#     origin2_transformed = origin2_transformed.flatten()
    
#     # --- 2. Leveling & Centering ---
#     v1, v2 = calib_points1[1] - calib_points1[0], calib_points1[2] - calib_points1[0]
#     normal = np.cross(v1, v2)
#     normal /= np.linalg.norm(normal)
    
#     target_up = np.array([0, 0, 1])
#     rot_axis = np.cross(normal, target_up)
#     if np.linalg.norm(rot_axis) > 1e-6:
#         rot_axis /= np.linalg.norm(rot_axis)
#         angle = np.arccos(np.clip(np.dot(normal, target_up), -1.0, 1.0))
#         leveling_R = R_tool.from_rotvec(rot_axis * angle).as_matrix()
#     else:
#         leveling_R = np.eye(3)

#     # Apply Leveling
#     pc1_l, pc2_l = pc1 @ leveling_R.T, pc2_transformed @ leveling_R.T
#     o1_l, o2_l = np.array([0,0,0]) @ leveling_R.T, origin2_transformed @ leveling_R.T
#     c1_l = calib_points1 @ leveling_R.T

#     # Centering
#     center = np.vstack([pc1_l, pc2_l]).mean(axis=0)
#     floor_z = c1_l[:, 2].mean()

#     # --- 3. Plotting Setup ---
#     fig = plt.figure(figsize=(12, 9), facecolor='black')
#     ax = fig.add_subplot(111, projection='3d', facecolor='black')
#     plt.subplots_adjust(left=0.2) # Make room for widgets on the left

#     # Create the scatter objects
#     sc1 = ax.scatter(pc1_l[:, 0]-center[0], pc1_l[:, 1]-center[1], pc1_l[:, 2]-floor_z, 
#                      s=1, color='red', alpha=0.4, label='Cam 1')
#     sc2 = ax.scatter(pc2_l[:, 0]-center[0], pc2_l[:, 1]-center[1], pc2_l[:, 2]-floor_z, 
#                      s=1, color='blue', alpha=0.4, label='Cam 2')
    
#     # Origins (Scatter + Text)
#     org1 = ax.scatter(o1_l[0]-center[0], o1_l[1]-center[1], o1_l[2]-floor_z, s=100, color='white', edgecolors='red')
#     txt1 = ax.text(o1_l[0]-center[0], o1_l[1]-center[1], o1_l[2]-floor_z, " Cam 1", color='white')
    
#     org2 = ax.scatter(o2_l[0]-center[0], o2_l[1]-center[1], o2_l[2]-floor_z, s=100, color='white', edgecolors='blue')
#     txt2 = ax.text(o2_l[0]-center[0], o2_l[1]-center[1], o2_l[2]-floor_z, " Cam 2", color='white')

#     # Calibration markers
#     cal = ax.scatter(c1_l[:, 0]-center[0], c1_l[:, 1]-center[1], c1_l[:, 2]-floor_z, s=150, color='lime', marker='X')

#     # Formatting
#     ax.set_axis_off()
#     limit = np.max(np.linalg.norm(pc1_l - center, axis=1))
#     ax.set_xlim(-limit, limit); ax.set_ylim(-limit, limit); ax.set_zlim(-limit*0.2, limit*1.2)

#     # --- 4. Toggle Widget ---
#     rax = plt.axes([0.05, 0.4, 0.12, 0.2], facecolor='#222222')
#     labels = ['Camera 1', 'Camera 2', 'Markers']
#     visibility = [True, True, True]
#     check = CheckButtons(rax, labels, visibility)

#     # Widget text color
#     for p in check.labels: p.set_color('white')

#     def func(label):
#         if label == 'Camera 1':
#             sc1.set_visible(not sc1.get_visible())
#             org1.set_visible(not org1.get_visible())
#             txt1.set_visible(not txt1.get_visible())
#         elif label == 'Camera 2':
#             sc2.set_visible(not sc2.get_visible())
#             org2.set_visible(not org2.get_visible())
#             txt2.set_visible(not txt2.get_visible())
#         elif label == 'Markers':
#             cal.set_visible(not cal.get_visible())
#         plt.draw()

#     check.on_clicked(func)
#     print("Use the checkboxes on the left to toggle point clouds.")
#     plt.show()
# # Run
# visualize_with_toggles(
#     cam_1_pc, cam_2_pc, calib_R, calib_t, calib_points_cam1
# )