"""
RPLidar Monitor

Define classes for com port monitor that initiates a thread continuously reading
data from serial_port


"""


import serial
import logging
import threading
import time

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *


class RPLidarMonitor(threading.Thread):
    """ A thread for monitoring a COM port.
    
        data_q:
            Queue for received data. Items in the queue are
            (data, timestamp) pairs, where data is a binary 
            string representing the received data, and timestamp
            is the time elapsed from the thread's start (in 
            seconds).
        
        port:
            The COM port to open. Must be recognized by the 
            system.
        
        port_baud/stopbits/parity: 
            Serial communication parameters
        
        port_timeout:
            The timeout used for reading the COM port. If this
            value is low, the thread will return data in finer
            grained chunks, with more accurate timestamps, but
            it will also consume more CPU.
    """
    
    
    def __init__(self, rplidar):

        logging.debug("Initializing rplidar_monitor thread.")

        threading.Thread.__init__(self)
        self.name = "rplidar_monitor"
        self.rplidar = rplidar
        self.serial_port = rplidar.serial_port

        self.raw_points = rplidar.raw_points
        self.raw_frames = rplidar.raw_frames
        self.current_frame = rplidar.current_frame
        
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
            while self.serial_port.inWaiting() < 5:
                time.sleep(.0001)

            raw_point = self.serial_port.read(5)
            point = rplidar_response_device_point_format.parse(raw_point)

            self.raw_points.put(raw_point)
            self.current_frame.add_point(point)
            
            # when syncbit == True, meaning a new frame starts
            # the existing frame, if any, is saved in raw_frames
            # then initialize a new empty frame
            if point.byte0.syncbit:
                if raw_frame:
                    self.raw_frames.append(raw_frame)
                    #logging.debug(len(raw_frame.points))
                    
                raw_frame = RPLidarRawFrame()
            
            raw_frame.add_point(raw_point)


    def join(self, timeout=0.1):

        self.alive.clear()
        self.stop_scan()        
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_monitor thread closed.")
