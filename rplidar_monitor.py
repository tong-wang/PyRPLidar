"""
RPLidar Monitor

Define classes related to the com port monitor, a thread continuously reading 
data from RPLidar on serial_port.

"""


import serial
import logging
import threading
import time

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *


class RPLidarRawFrame(object):
    """Raw frame from the RPLidar scan.
    
    Save the timestamp and raw points of a complete frame. This is for archiving
    the data.
    
    Attibutes:
        timestamp: when the frame is initiated and recorded.

        points: a list of points (in raw RPLidar binary format) of a complete 
        frame, starting from the point with syncbit == 1.
    """
    
    def __init__(self):
        self.timestamp = time.time()
        self.points = list()
        #self.isComplete = False
    
    def add_point(self, point):
        """append new point to the points list."""
        self.points.append(point)


class RPLidarFrame(object):
    """A processed frame with readily usable coordinates.
    
    Contains a moving window (implemented by deques with maxlen=360) of the most
    recent 360 points, each being converted from raw point data to both 
    Cartesian and polar coordinates.
    
    This is mainly for real-time visualization of the points.
    
    Attributes:
        angle_d: a deque keeping angle in degrees
        
        angle_r: a deque keeping angle in radians
        
        distance: a deque keeping distance in millimeters
        
        x: a deque keeping x coordinate in millimeters
        
        y: a deque keeping y coordinate in millimeters
        
    Methods:
        add_point(): 
    """
    
    def __init__(self):

        #self.updated = False
        self.angle_d = deque(maxlen = 360)
        self.angle_r = deque(maxlen = 360)
        self.distance = deque(maxlen = 360)
        self.x = deque(maxlen = 360)
        self.y = deque(maxlen = 360)
        
    def add_point(self, point):
        """add a parsed point into the deques
        
        Args:
            point: a parsed point in rplidar_response_device_point_format.
        """
        
        #self.updated = True
        distance = point.distance_q2 / 4.0
        angle_d = ((point.angle_highbyte<<7) | point.byte1.angle_lowbyte) / 64.0
        angle_r = np.radians(angle_d)
        
        self.angle_d.append(angle_d)
        self.angle_r.append(angle_r)
        self.distance.append(distance)
        self.x.append(distance * np.sin(angle_r))
        self.y.append(distance * np.cos(angle_r))


class RPLidarMonitor(threading.Thread):
    """ A thread for monitoring RPLidar on a COM port.
    
        When initiated, the thread will send SCAN command to RPLidar, 
        continously read raw data feed from RPLidar, and save the data into 
        Queues (including self.rplidar.raw_points, self.rplidar.raw_frames, and 
        self.rplidar.current_frame) for further processing.
    
        Attributes:
            name: thread name.
        
            rplidar: the parent rplidar instance.
        
            alive: the monitor thread is working when alive is set() and stops 
            when alive is clear().
    """
    
    
    def __init__(self, rplidar):

        logging.debug("Initializing rplidar_monitor thread.")

        threading.Thread.__init__(self)
        self.name = "rplidar_monitor"
        self.rplidar = rplidar
        self.alive = threading.Event()
        self.alive.set()
        
        logging.debug("rplidar_monitor thread initialized.")
        
        
    def start_scan(self):
        """Send SCAN command to RPLidar."""
        
        self.rplidar.send_command(RPLIDAR_CMD_SCAN)

        if self.rplidar.response_header() != RPLIDAR_ANS_TYPE_MEASUREMENT:
            raise RPLidarError("RESULT_INVALID_MEASUREMENT_HEADER")

        self.rplidar.isScanning = True
        logging.debug("Start scanning.")


    def stop_scan(self):
        """Send STOP command to RPLidar."""
        
        self.rplidar.send_command(RPLIDAR_CMD_STOP)

        # it seems to be very important to have a pause here 
        # until the STOP command is executed by RPLidar
        time.sleep(0.1) 
        
        self.rplidar.isScanning = False;
        logging.debug("Stop scanning.")
    

    def run(self):
        """Main thread function.
        
        Continuously reads raw data feed (raw_point) from RPLidar and 
        modifies:
            raw_points: raw_point is directly put into it;
            
            current_frame: raw_point is first parsed then added into it;
            
            raw_frames: everytime when there is a point with syncbit==True, the 
            previous raw_frame is put into raw_frames, a new raw_frame instance 
            is initiated. The raw_point is added into raw_frame.
        
        """

        self.start_scan()
        
        raw_frame = None
        
        while self.alive.isSet():
            while self.rplidar.serial_port.inWaiting() < 5:
                time.sleep(.0001)

            raw_point = self.rplidar.serial_port.read(5)
            point = rplidar_response_device_point_format.parse(raw_point)

            self.rplidar.raw_points.put(raw_point)
            self.rplidar.current_frame.add_point(point)
            
            # when syncbit == True, meaning a new frame starts
            # the existing frame, if any, is saved in raw_frames
            # then initialize a new empty frame
            if point.byte0.syncbit:
                if raw_frame:
                    self.rplidar.raw_frames.put(raw_frame)
                    logging.debug("raw_frames qsize: %d, raw_frame length: %d.",
                        self.rplidar.raw_frames.qsize(), len(raw_frame.points))

                                        
                raw_frame = RPLidarRawFrame()
            
            raw_frame.add_point(raw_point)


    def join(self, timeout=0.1):

        self.alive.clear()
        self.stop_scan()        
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_monitor thread closed.")
