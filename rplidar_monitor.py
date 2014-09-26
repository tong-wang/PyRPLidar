'''
RPLidar Monitor

Define classes for com port monitor that initiates a thread continuously reading data from serial_port


'''

import Queue
import numpy as np
import time
import threading
import serial

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *



toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x])



class RPLidarFrame(object):
    def __init__(self):
        self.cur_data = np.zeros((360, 5))
        self.has_new_data = False
    
    def add_data(self, data):
        self.cur_data[np.floor(data.angleD)] = [data.angle, data.distance, data.angleD, data.X, data.Y]
        self.has_new_data = True
    
    def read_data(self):
        self.has_new_data = False
        return self.cur_data
        
        
class RPLidarPoint(object):
    def __init__(self, rawPoint):

        self.timestamp = time.clock()
        self.raw = rawPoint
        
        _parsed = rplidar_response_device_point_format.parse(rawPoint)
        
        self.syncbit = _parsed.Byte0.syncbit
        self.syncbit_inverse = _parsed.Byte0.syncbit_inverse
        self.quality = _parsed.Byte0.sync_quality
        
        self.check_bit = _parsed.Byte1.check_bit
        self.angleD = ((_parsed.angle_highbyte << 7) | _parsed.Byte1.angle_lowbyte) / 64.0
        self.angle = np.radians(self.angleD)
        
        self.distance = _parsed.distance_q2 / 4.0
        
        self.X = self.distance * np.sin(self.angle)
        self.Y = self.distance * np.cos(self.angle)
        
        

    



class RPLidarMonitor(threading.Thread):
    """ A thread for monitoring a COM port. The COM port is 
        opened when the thread is started.
    
        data_q:
            Queue for received data. Items in the queue are
            (data, timestamp) pairs, where data is a binary 
            string representing the received data, and timestamp
            is the time elapsed from the thread's start (in 
            seconds).
        
        error_q:
            Queue for error messages. In particular, if the 
            serial port fails to open for some reason, an error
            is placed into this queue.
        
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
        threading.Thread.__init__(self)
        
        self.rplidar = rplidar
        self.serial_port = rplidar.serial_port
        self.serial_arg = rplidar.serial_arg
        self.data_q = rplidar.data_q
        self.error_q = rplidar.error_q
        
        self.alive = threading.Event()
        self.alive.set()
        
        print "[rplidar_monitor]: initiated."
        

        
    def openSerialPort(self):        
        try:
            if self.serial_port: 
                self.serial_port.close()
                print "[rplidar_monitor]: close serial port."
            self.serial_port = serial.Serial(**self.serial_arg)
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
            print "[rplidar_monitor]: open serial port."
        except serial.SerialException, e:
            print e
            self.error_q.put(e.message)
            return


    def startScan(self):
        
        if not self.serial_port:
            self.openSerialPort()
        
        self.rplidar.sendCommand(RPLIDAR_CMD_SCAN)
        print "[rplidar_monitor]: start scanning."
        
        if self.rplidar.waitResponseHeader() != RPLIDAR_ANS_TYPE_MEASUREMENT:
            raise RPLidarError("RESULT_INVALID_MEASUREMENT_HEADER")

    

    
    def run(self):

        # Restart the clock
        time.clock()

        self.startScan()
           
        
        while self.alive.isSet():
            
            while self.serial_port.inWaiting()<5:
                time.sleep(.0001)

            rawPoint = self.serial_port.read(5)
            point = RPLidarPoint(rawPoint)
            self.data_q.put(point)
            
        # clean up
        if self.serial_port:
            self.serial_port.close()

    
    
    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
        print "[rplidar_monitor]: thread closed."
