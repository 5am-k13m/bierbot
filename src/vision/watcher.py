from pykinect import nui
import numpy as np
import cv2

class Watcher:
    def __init__(self,
                 display=True,
                 index=0,
                 fx=524, fy=524,
                 cx=316.7, cy=238.5,
                 kc=[0.2402, -0.6861, -0.0015, 0.0003],
                 ir_toggle=False):

        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.kc = kc
        self.index = index
        self.display = display

        self.video = np.empty((480,640,4),np.uint8)
        self.depth = np.empty((480,640),np.uint16)
        self.ir = np.empty((480,640),np.uint16)

        self.video_frame_counter = 0
        self.depth_frame_counter = 0
        self.ir_frame_counter = 0

        self.ir_toggle = ir_toggle
        self.kinect = nui.Runtime(index=index)
        self.stream_open = False

        # Start streams
        if not ir_toggle:
            self.start_stream()
        else:
            self.start_ir()

        # Display windows if requested
        if self.display:
            self.create_windows()

    # -------------------------------------------------
    # NEW METHOD 1: Project 3D → 2D pixels
    # -------------------------------------------------
    def _project_points(self, pts_cam):
        """
        pts_cam: Nx3 array in CAMERA COORDINATES
        Returns: u,v pixel coordinates
        """

        if pts_cam is None or len(pts_cam) == 0:
            return np.zeros((0,2), dtype=np.int32)

        fx, fy = self.fx, self.fy
        cx, cy = self.cx, self.cy

        X = pts_cam[:, 0]
        Y = pts_cam[:, 1]
        Z = pts_cam[:, 2]

        if len(Z) == 0:
            return np.zeros((0,2), dtype=np.int32)

        u = (fx * X / Z + cx).astype(np.int32)
        v = (fy * Y / Z + cy).astype(np.int32)

        return np.vstack([u, v]).T

    # -------------------------------------------------
    # NEW METHOD 2: Draw on RGB frame
    # -------------------------------------------------
    def overlay_points(self, frame, pts_cam, is_floor=True, point_size=2):
        """
        Draws 3D camera-space points onto the RGB frame.
        
        - Floor points = blue
        - Obstacle points = red
        """

        if frame is None or pts_cam is None:
            return frame

        pts_2d = self._project_points(pts_cam)
        if len(pts_2d) == 0:
            return frame

        color = (255,0,0) if is_floor else (0,0,255)  # BGR

        h, w = frame.shape[:2]

        for (u, v) in pts_2d:
            if 0 <= u < w and 0 <= v < h:
                cv2.circle(frame, (u, v), point_size, color, -1)

        return frame

    # ------------------ Stream control ------------------
    def start_stream(self):
        if not self.stream_open:
            self.kinect = nui.Runtime(index=self.index)
            self.kinect.video_frame_ready += self.video_handler_function
            self.kinect.video_stream.open(
                nui.ImageStreamType.Video, 2,
                nui.ImageResolution.Resolution640x480,
                nui.ImageType.Color)

            self.kinect.depth_frame_ready += self.depth_handler_function
            self.kinect.depth_stream.open(
                nui.ImageStreamType.Depth, 2,
                nui.ImageResolution.Resolution640x480,
                nui.ImageType.Depth)

            self.stream_open = True

    def stop_stream(self):
        if self.stream_open:
            self.kinect.close()
            self.stream_open = False

    def start_ir(self):
        if not self.stream_open:
            self.kinect = nui.Runtime(index=self.index)
            self.kinect.depth_frame_ready += self.ir_handler_function
            self.kinect.depth_stream.open(
                nui.ImageStreamType.Depth, 2,
                nui.ImageResolution.Resolution640x480,
                nui.ImageType.Infrared)
            self.stream_open = True

    # ------------------ Display ------------------
    def create_windows(self):
        if not self.ir_toggle:
            cv2.namedWindow(f'KINECT {self.index} Video Stream', cv2.WINDOW_AUTOSIZE)
            cv2.namedWindow(f'KINECT {self.index} Depth Stream', cv2.WINDOW_AUTOSIZE)
        else:
            cv2.namedWindow(f'KINECT {self.index} IR Stream', cv2.WINDOW_AUTOSIZE)

    def destroy_windows(self):
        if not self.ir_toggle:
            cv2.destroyWindow(f'KINECT {self.index} Video Stream')
            cv2.destroyWindow(f'KINECT {self.index} Depth Stream')
        else:
            cv2.destroyWindow(f'KINECT {self.index} IR Stream')

    # ------------------ Handlers ------------------
    def video_handler_function(self, frame):
        frame.image.copy_bits(self.video.ctypes.data)
        self.video_frame_counter += 1

    def depth_handler_function(self, frame):
        frame.image.copy_bits(self.depth.ctypes.data)
        self.depth_frame_counter += 1

    def ir_handler_function(self, frame):
        frame.image.copy_bits(self.ir.ctypes.data)
        self.ir_frame_counter += 1

    # ------------------ Close ------------------
    def close(self):
        self.destroy_windows()
        self.kinect.close()
