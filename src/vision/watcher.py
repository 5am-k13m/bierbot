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
