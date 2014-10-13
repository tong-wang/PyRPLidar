"""
RPLidar Processor

Define classes for a thread that continuously processing data from rplidar_monitor


"""

import serial
import logging
import threading
import time
import Queue
from collections import deque
import copy
import numpy as np

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *


class RPLidarProcessor(threading.Thread):
    """ A thread for processing data from rplidar_monitor
    """

        
    def __init__(self, rplidar):

        logging.debug("Initializing rplidar_processor thread.")

        threading.Thread.__init__(self)
        self.name = "rplidar_processor"
        self.rplidar = rplidar
        self.serial_port = rplidar.serial_port
        
        #self.raw_points = rplidar.raw_points
        self.raw_frames = rplidar.raw_frames
        #self.frames = rplidar.frames
        self.curFrame = rplidar.curFrame
        
        self.alive = threading.Event()
        self.alive.set()
        
        logging.debug("rplidar_processor thread initialized.")
        

    
    def run(self):

        while self.alive.isSet():
            if self.curFrame.timestamp < self.raw_frames[-1].timestamp: 
                raw_frame = self.raw_frames[-1]
                self.curFrame = RPLidarFrame(raw_frame)

    
    
    def join(self, timeout=None):
        
        self.alive.clear()
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_processor thread closed.")
