""" 
sss
"""

import serial
import math
import Queue
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

from rplidar_protocol import *
from com_monitor import ComMonitorThread




toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x])



class FrameData(object):
    def __init__(self):
        self.cur_data = deque(maxlen=360)
        self.has_new_data = False
    
    def add_data(self, data):
        self.cur_data.append(data)
        self.has_new_data = True
    
    def read_data(self):
        self.has_new_data = False
        return self.cur_data


class RPLidar(object):

    
    def __init__(self, portname, baudrate=115200, timeout=0.01):
        
        #private flags
        self._isConnected = False
        
        #init serial port
        self.serial_port = None
        self.portname = portname
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_arg = dict( port=portname,
                                baudrate=baudrate,
                                stopbits=serial.STOPBITS_ONE,
                                parity=serial.PARITY_NONE,
                                timeout=timeout)
        
        #init com_monitor
        self.com_monitor = None
        self._isScanning = False
        self.data_q = Queue.Queue()
        self.error_q = Queue.Queue()
        
        self.frameData = FrameData()
        self.environmentFrame = np.empty([361,2]);
        self.environmentFrame.fill(100000)
        self.environmentX = np.empty(361)
        self.environmentY = np.empty(361)
        
        #setup plot
        plt.ion()
        self.figure = plt.figure(figsize=(6, 6), dpi=160, facecolor='w', edgecolor='k')
        self.ax = self.figure.add_subplot(111)
        self.lines, = self.ax.plot([],[], linestyle='none', marker='.', markersize=3, markerfacecolor='blue')
        self.environment, = self.ax.plot([],[], linestyle='none', marker='.', markersize=3, markerfacecolor='red')
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.grid()
        
        
    def connect(self):
        if not self._isConnected:
            try:
                self.serial_port = serial.Serial(**self.serial_arg)
                #self.serial_port.flushInput()
                #self.serial_port.flushOutput()
                self._isConnected = True
            except serial.SerialException, e:
                self.error_q.put(e.message)
                

    def disconnect(self):
        if self._isConnected:
            try:
                self.stop() #!!!!!!!!!!!!!!!!!!!!!!!!
                self.serial_port.close()
                self._isConnected = False
            except serial.SerialException, e:
                self.error_q.put(e.message)
            


    def stop(self):
        
        #stop scanning
        sendCommand(self.serial_port, RPLIDAR_CMD_STOP)

        time.sleep(0.1) #it seems to be very important to have a pause here until the STOP command is executed by RPLidar
        
        #stop monitor thread
        self.stopMonitor()

    
    def reset(self):
        sendCommand(self.serial_port, RPLIDAR_CMD_RESET) #reset
        

    def getDeviceInfo(self):
        "Obtain hardware information about RPLidar"
        
        
        #disconnect then re-connect serial port in order to clear buffer
        #self.disconnect()
        #self.connect()
        
        self.stop()
        self.serial_port.flushInput()
        

        sendCommand(self.serial_port, RPLIDAR_CMD_GET_DEVICE_INFO) #get device info
        

        if waitResponseHeader(self.serial_port) == RPLIDAR_ANS_TYPE_DEVINFO:
            rawInfo = self.serial_port.read(rplidar_response_device_info_format.sizeof())
            parsed = rplidar_response_device_info_format.parse(rawInfo)
        
            return {'model': parsed.model, 
                    'firmware_version_major': parsed.firmware_version_major, 
                    'firmware_version_minor': parsed.firmware_version_minor, 
                    'firmware_version': str(parsed.firmware_version_major) + "." + str(parsed.firmware_version_minor),
                    'hardware_version': parsed.hardware_version,
                    'serialno': toHex(parsed.serialnum).upper()
                    }

        else:
            raise RPLidarError("RESULT_INVALID_ANS_TYPE")


    def getHealth(self):
        "Obtain health information about RPLidar"

        #disconnect then re-connect serial port in order to clear buffer
        #self.disconnect()
        #self.connect()
        
        self.stop()
        self.serial_port.flushInput()
        

        sendCommand(self.serial_port, RPLIDAR_CMD_GET_DEVICE_HEALTH) #get device health

        if waitResponseHeader(self.serial_port) == RPLIDAR_ANS_TYPE_DEVHEALTH:
            rawInfo = self.serial_port.read(rplidar_response_device_health_format.sizeof())
            parsed = rplidar_response_device_health_format.parse(rawInfo)
                
            return {'status': parsed.status, 
                    'error_code': parsed.error_code}
        else:
            raise RPLidarError("RESULT_INVALID_ANS_TYPE")
           
                                          
    
    
    def set_actions_enable_state(self):
        if self.portname == '':
            start_enable = stop_enable = False
        else:
            start_enable = not self._isScanning
            stop_enable = self._isScanning
        


    def startMonitor(self):
        """ Start the monitor: com_monitor thread and the update
            timer
        """
        
        if self.portname == '':
            raise RPLidarError("EMPTY SERIAL PORT NAME")
        
        if not self._isScanning:
        
            self.com_monitor = ComMonitorThread(
                self.data_q,
                self.error_q,
                self.serial_arg)
            self.com_monitor.start()
            
            try: 
                com_error = self.error_q.get(True, 0.01)
            except Queue.Empty: 
                com_error = None
    
            if com_error is not None:
                print self, 'ComMonitorThread error', com_error
                self.com_monitor = None
    
            self._isScanning = True
            self.set_actions_enable_state()
                    
            print 'Monitor running:'
    
    

    def stopMonitor(self):
        """ Stop the monitor
        """
        if self._isScanning:
            self._isScanning = False;
            self.com_monitor.join(0.01)
            self.com_monitor = None

            #self.set_actions_enable_state()
            print 'Monitor stopped.\n'
        
    
            
    def readFrameFromQueue(self):
        """ Called periodically by the update timer to read data
            from the serial port.
        """
        i = 0
        while i < 360:
            try: 
                qdata = self.data_q.get(True, 0.01)
            except Queue.Empty: 
                continue
            
            data = dict(timestamp=qdata[1], data=qdata[0])

            self.frameData.add_data(data)
            
            i = i+1
    
    

    def updatePlot(self):
        """ Updates the state of the monitor window with new 
            data. The livefeed is used to find out whether new
            data was received since the last update. If not, 
            nothing is updated.
        """
        if self.frameData.has_new_data:
            _framedata = self.frameData.read_data()
        
            _x = []
            _y = []
            
            for _point in list(_framedata):
                if _point['data']['distance'] < self.environmentFrame[round(_point['data']['angle']),1] - 50:
                    _x.append(_point['data']['distance'] * math.sin((_point['data']['angle']/180.0)*math.pi))
                    _y.append(_point['data']['distance'] * math.cos((_point['data']['angle']/180.0)*math.pi))
    

            self.lines.set_xdata(_x)
            self.lines.set_ydata(_y)
            self.figure.canvas.draw()
            
    
    



        
            
                
        
        
        


if __name__ == "__main__":
    
    rplidar = RPLidar("/dev/tty.SLAB_USBtoUART")
    
    rplidar.connect()
   
    try:
        print rplidar.getDeviceInfo()
    except RPLidarError, e:
        print e


    try:
        print rplidar.getHealth()
    except RPLidarError, e:
        print e


    rplidar.startMonitor()

    for i in range(1000):
        rplidar.readFrameFromQueue()
        rplidar.updatePlot()
  
        
    rplidar.stopMonitor()
    
    rplidar.disconnect()


    
    
        
            
                    
    rplidar = None
  
