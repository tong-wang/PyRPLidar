import Queue
import time
import threading
import serial

from rplidar_serial  import *






class RPLidarMonitorThread(threading.Thread):
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
    
    
    def __init__(self, data_q, error_q, serial_arg):
        threading.Thread.__init__(self)
        
        self.serial_port = None
        self.serial_arg = serial_arg
        self.data_q = data_q
        self.error_q = error_q
        
        self.alive = threading.Event()
        self.alive.set()
        
        

        
    def openSerialPort(self):        
        try:
            if self.serial_port: 
                self.serial_port.close()
            self.serial_port = serial.Serial(**self.serial_arg)
            self.serial_port.flushInput()
            self.serial_port.flushOutput()
        except serial.SerialException, e:
            print e
            self.error_q.put(e.message)
            return


    def startScan(self):
        
        if not self.serial_port:
            self.openSerialPort()
        
        sendCommand(self.serial_port, RPLIDAR_CMD_SCAN)
        
        if waitResponseHeader(self.serial_port) != RPLIDAR_ANS_TYPE_MEASUREMENT:
            raise RPLidarError("RESULT_INVALID_MEASUREMENT_HEADER")

    

    
    def run(self):

        # Restart the clock
        time.clock()

        #self.openSerialPort()
        self.startScan()

        
        while self.alive.isSet():
            
            while self.serial_port.inWaiting()<5:
                time.sleep(.0001)

            rawPoint = self.serial_port.read(5)
            parsed = rplidar_response_device_point_format.parse(rawPoint)
            point = dict(start=parsed.Byte0.syncbit, angle=parsed.angle, distance=parsed.distance, check=parsed.Byte1.check_bit, quality=parsed.Byte0.sync_quality)
            
            timestamp = time.clock()
            self.data_q.put((point, timestamp))
            
        # clean up
        if self.serial_port:
            self.serial_port.close()

    
    
    def join(self, timeout=None):
        self.alive.clear()
        threading.Thread.join(self, timeout)
