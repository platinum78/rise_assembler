import numpy as np
import pyrealsense2 as rs

class RealSenseHandler:
    def __init__(self):
        # Initialize connection with Realsense camera
        self.pipeline = rs.pipeline()
        self.pipeline.start()
        self.frames = self.pipeline.wait_for_frames()

        # NumPy arrays that store data
        self.depth_frame = None
        self.color_frame = None
    
    def get_depth_frame(self):
        depth_frame_obj = self.frames.get_depth_frame()
        self.depth_frame = np.array(depth_frame_obj.as_frame().get_data())
    
    def get_color_frame(self):
        color_frame_obj = self.frames.
        