from __future__ import annotations

import cv2
import numpy as np
from typing import *
from dataclasses import dataclass, field

@dataclass
class MotionInfo:
    points: List[np.ndarray] = field(default_factory=list)
    headtmap: Optional[np.ndarray] = None
    status: bool = False
    
class MotionDetector(MotionInfo):
    # Number of frames to pass before changing the frame to compare the current
    # frame against
    FRAMES_TO_PERSIST: int = 10

    # Minimum boxed area for a detected motion to count as actual motion
    # Use to filter out noise or small objects
    MIN_SIZE_FOR_MOVEMENT: int = 2000

    # Minimum length of time where no motion is detected it should take
    #(in program cycles) for the program to declare that there is no movement
    MOVEMENT_DETECTED_PERSISTENCE: int = 50
    
    def __init__(self, display_size: Optional[tuple] = (1280, 720)):
        MotionInfo.__init__(self)
        self.first_frame = None
        self.next_frame = None
        self.display_size = display_size
        
        self.__frame_counter = 0
        self.__movement_counter = 0

    def __update_status(self, flag: bool) -> None:
        # The moment something moves momentarily, reset the persistent movement timer.
        if flag == True:
            self.__movement_counter = MotionDetector.MOVEMENT_DETECTED_PERSISTENCE
        
        # As long as there was a recent transient movement, say a movement was detected    
        if self.__movement_counter > 0:
            self.status = True
            self.__movement_counter -= 1
        else:
            self.status = False

    @staticmethod
    def findMotionContours(first_frame: np.ndarray, next_frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Find motion contours between two frames.

        Args:
            first_frame (numpy.ndarray): First frame.
            next_frame (numpy.ndarray): Next frame.

        Returns:
            tuple: A tuple containing the difference frame and a list of motion information.
        """
        motion_info = []
        # Compare the two frames, find the difference
        frame_delta = cv2.absdiff(first_frame, next_frame)
        # mask = cv2.threshold(frame_delta, 25, 255, cv2.THRESH_BINARY)[1]
        mask = cv2.adaptiveThreshold(frame_delta, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                cv2.THRESH_BINARY_INV, 11, 3)
        mask = cv2.medianBlur(mask, 3)
        
        # Fill in holes via dilate(), and find contours of the thesholds
        mask = cv2.dilate(mask, None, iterations=2)
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)
        
        for c in cnts:
            # Save the coordinates of all found contours
            (x, y, w, h) = cv2.boundingRect(c)
            # If the contour is too small, ignore it, otherwise, there's transient movement
            if cv2.contourArea(c) > MotionDetector.MIN_SIZE_FOR_MOVEMENT:
                motion_info.append([x, y, w, h])
        frame_delta = cv2.cvtColor(frame_delta, cv2.COLOR_GRAY2BGR)
        return frame_delta, np.array(motion_info, dtype=np.float64)

    def update(self, srcimg: np.ndarray) -> bool:
        """
        Update motion detection with a new frame.

        Args:
            srcimg (numpy.ndarray): Input frame.
        """
        width, height, _ = srcimg.shape
        resizeimg = cv2.resize(srcimg, self.display_size)
        gray = cv2.cvtColor(resizeimg, cv2.COLOR_BGR2GRAY)
        # Blur it to remove camera noise (reducing false positives)
        kernel_size = int(21/(resizeimg.shape[0]/300)), int(21/(resizeimg.shape[1]/500))
        kernel_size = (kernel_size[0] + 1 if kernel_size[0] % 2 == 0 else kernel_size[0],
                       kernel_size[1] + 1 if kernel_size[1] % 2 == 0 else kernel_size[1])
        gray = cv2.GaussianBlur(gray, kernel_size, 0)
        if self.first_frame is None: 
            self.first_frame = gray 

        # Otherwise, set the first frame to compare as the previous frame
        # But only if the counter reaches the appriopriate value
        # The delay is to allow relatively slow motions to be counted as large
        # motions if they're spread out far enough
        self.__frame_counter += 1
        if self.__frame_counter > MotionDetector.FRAMES_TO_PERSIST:
            self.__frame_counter = 0
            self.first_frame = self.next_frame
        self.next_frame = gray

        _headtmap, _points = self.findMotionContours(self.first_frame, self.next_frame)
        self.headtmap = cv2.resize(_headtmap, (height, width))
        if _points.shape[0] > 0:
            _points[:, [0, 2]] *= width/self.display_size[1]
            _points[:, [1, 3]] *= height/self.display_size[0]
            self.points = _points.astype(int)
        else :
            self.points = _points
        self.__update_status(len(self.points) > 0)
        return self.status
    
    def DrawMotionOnFrame(self, frame: np.ndarray) -> None:
        """
        Draw motion on the input frame.

        Args:
            frame (numpy.ndarray): Input frame.
        """
        if len(self.points):
            overlay = frame.copy()
            for rect in self.points:
                x, y, w, h = rect
                # Draw a rectangle on the overlay
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (255, 255, 255), -1)  # Filled rectangle

            # Apply the overlay
            alpha = 0.2
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
    def DrawMotionHeatmap(self, window_name: str = "Motion Display") -> None:
        """
        Draw motion heatmap.

        Args:
            window_name (str): Name of the window to display the heatmap.
        """
        if not isinstance(self.headtmap, np.ndarray): 
            return
        
        if self.status:
            color = (0, 255, 0)
            text = "Movement Detected | Timer: " + str(self.__movement_counter)
        else:
            color = (0, 0, 255)
            text = "No Movement Detected"
        cv2.namedWindow(window_name)
        if self.display_size != None:
            _headtmap = cv2.resize(self.headtmap, self.display_size) if self.display_size != None else self.headtmap
        cv2.putText(_headtmap, str(text), (10,35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)
        cv2.imshow(window_name, _headtmap)