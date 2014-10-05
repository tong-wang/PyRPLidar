"""
RPLidar Monitor

Define the class for com port monitor, a thread continuously reading data from
RPLidar on serial_port.

"""


import serial
import logging
import threading
import time

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *


class RPLidarMonitor(threading.Thread):
    """ A thread for monitoring RPLidar on a COM port.
    
        When initiated, the thread will send SCAN command to RPLidar, 
        continously read raw data feed from RPLidar, and save the data into 
        Queues (including self.rplidar.raw_points, self.rplidar.raw_frames, and 
        self.rplidar.current_frame) for further processing.
    
        name:
            Thread name.
        
        rplidar:
            The parent rplidar instance.
        
        alive: 
            The monitor thread is working when alive is set() and stops when 
            alive is clear().
        
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
        
        self.rplidar.send_command(RPLIDAR_CMD_SCAN)

        if self.rplidar.response_header() != RPLIDAR_ANS_TYPE_MEASUREMENT:
            raise RPLidarError("RESULT_INVALID_MEASUREMENT_HEADER")

        self.rplidar.isScanning = True
        logging.debug("Start scanning.")


    def stop_scan(self):
        
        self.rplidar.send_command(RPLIDAR_CMD_STOP)

        # it seems to be very important to have a pause here 
        # until the STOP command is executed by RPLidar
        time.sleep(0.1) 
        
        self.rplidar.isScanning = False;
        logging.debug("Stop scanning.")
    

    def run(self):

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
