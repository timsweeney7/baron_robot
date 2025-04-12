#!/usr/bin/python3

from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import time
import matplotlib.pyplot as plt
import cv2
import numpy as np

class TrackingOutput(FileOutput):
    
    # Define lower and upper bounds for Hue (70-80), with full Saturation and Value ranges
    lower_bound = np.array([60, 50, 50])  # Lower bound (Hue=70, min Sat & Val)
    upper_bound = np.array([80, 255, 255])  # Upper bound (Hue=80, max Sat & Val)
    
    BGR_RED = (0,0,255)
    RGB_RED = (255,0,0)
    
    def __init__(self, filename):
        super().__init__(filename)
        
    def outputframe(self, frame, keyframe=None, timestamp=None):
        
        print(np.shape(frame))
        print(self._fileoutput)
        print(self.recording)
        
        if self._fileoutput is not None and self.recording:
            if self._firstframe:
                if not keyframe:
                    return
                else:
                    self._firstframe = False
        
        # Convert frame to NumPy array
        frame_array = np.frombuffer(frame, dtype=np.uint8)
        frame_array = cv2.imdecode(frame_array, cv2.IMREAD_COLOR)
        hsv_image = cv2.cvtColor(frame_array, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv_image, self.lower_bound, self.upper_bound)
        contours, _ = cv2.findContours(mask, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_NONE)
        (x_axis, y_axis), radius = cv2.minEnclosingCircle(points=contours[0])
        radius = int(radius)
        circle_center = (int(x_axis), int(y_axis))
        cv2.circle(frame_array, center=circle_center, radius=radius, color=RGB_RED, thickness=2)
        
        # Encode and write the processed frame
        success, encoded_frame = cv2.imencode(".jpg", frame_array)
        if success:
            super().outputframe(encoded_frame.tobytes(), keyframe, timestamp)
        
        

if __name__ == "__main__":
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration()
    picam2.configure(video_config)

    encoder = H264Encoder(10000000)
    output = TrackingOutput('test.h264')

    picam2.start_recording(encoder, output)
    time.sleep(3)
    picam2.stop_recording()