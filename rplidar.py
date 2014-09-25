""" 
RPLidar Python Driver

...
...

"""

import serial
import math
import Queue
import numpy as np
import matplotlib.pyplot as plt

from rplidar_monitor import *










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
        
        #init monitor
        self.monitor = None
        self._isScanning = False
        self.data_q = Queue.Queue()
        self.error_q = Queue.Queue()
        
        self.frameData = RPLidarFrame()
        
        
        
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
        self.sendCommand(RPLIDAR_CMD_STOP)

        time.sleep(0.1) #it seems to be very important to have a pause here until the STOP command is executed by RPLidar
        
        #stop monitor thread
        self.stopMonitor()

    
    def reset(self):
        self.sendCommand(RPLIDAR_CMD_RESET) #reset

        

    def sendCommand(self, command):
        
        cmdBytes = rplidar_command_format.build(Container(syncByte = RPLIDAR_CMD_SYNC_BYTE, cmd_flag = command))
        self.serial_port.write(cmdBytes)
    
    
    
    
    def waitResponseHeader(self, timeout=1):
        
        _startTime = time.time()
        
        while time.time() < _startTime + timeout:
            if self.serial_port.inWaiting() < rplidar_response_header_format.sizeof():
                #print serial_port.inWaiting()
                time.sleep(0.01)
            else:
                _raw = self.serial_port.read(rplidar_response_header_format.sizeof())
                _parsed = rplidar_response_header_format.parse(_raw)
                #print _parsed
                
                if (_parsed.syncByte1 != RPLIDAR_ANS_SYNC_BYTE1) or (_parsed.syncByte2 != RPLIDAR_ANS_SYNC_BYTE2):
                    raise RPLidarError("RESULT_INVALID_ANS_HEADER")
                else:
                    return _parsed.type
    
        raise RPLidarError("RESULT_READING_TIMEOUT")
    
    
    
    
    def getDeviceInfo(self):
        "Obtain hardware information about RPLidar"
        
        
        #disconnect then re-connect serial port in order to clear buffer
        #self.disconnect()
        #self.connect()
        
        self.stop()
        self.serial_port.flushInput()
        

        self.sendCommand(RPLIDAR_CMD_GET_DEVICE_INFO) #get device info
        

        if self.waitResponseHeader() == RPLIDAR_ANS_TYPE_DEVINFO:
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
        

        self.sendCommand(RPLIDAR_CMD_GET_DEVICE_HEALTH) #get device health

        if self.waitResponseHeader() == RPLIDAR_ANS_TYPE_DEVHEALTH:
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
        """ Start the monitor: monitor thread and the update
            timer
        """
        
        if self.portname == '':
            raise RPLidarError("EMPTY SERIAL PORT NAME")
        
        if not self._isScanning:
        
            self.monitor = RPLidarMonitor(self)
            self.monitor.start()
            
            try: 
                com_error = self.error_q.get(True, 0.01)
            except Queue.Empty: 
                com_error = None
    
            if com_error is not None:
                print self, 'RPLidarMonitorThread error', com_error
                self.monitor = None
    
            self._isScanning = True
            self.set_actions_enable_state()
                    
            print '[rplidar]: Monitor running:'
    
    

    def stopMonitor(self):
        """ Stop the monitor
        """
        if self._isScanning:
            self._isScanning = False;
            self.monitor.join(0.01)
            self.monitor = None

            #self.set_actions_enable_state()
            print '[rplidar]: Monitor stopped.\n'
        
    
            
    def readFrameFromQueue(self):
        """ 
        read a whole frame from the data queue
        """
        i = 0
        while i < 360:
            try: 
                qdata = self.data_q.get(True, 0.01)
            except Queue.Empty: 
                continue
            
            self.frameData.add_data(qdata)
                
            i = i+1
    
    
    def initXYPlot(self):
        """
        setup an XY plot canvas
        """
        
        plt.ion()
        self.figure = plt.figure(figsize=(6, 6), dpi=160, facecolor='w', edgecolor='k')
        self.ax = self.figure.add_subplot(111)
        self.lines, = self.ax.plot([],[], linestyle='none', marker='.', markersize=3, markerfacecolor='blue')
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.grid()
    
    
    
    def updateXYPlot(self):
        """
        re-draw the XY plot with new frameData
        """

        if self.frameData.has_new_data:
            _framedata = self.frameData.read_data()
        
            _x = []
            _y = []
            
            for _point in list(_framedata):
                _x.append(_point.distance * math.sin(_point.angle))
                _y.append(_point.distance * math.cos(_point.angle))
    

            self.lines.set_xdata(_x)
            self.lines.set_ydata(_y)
            self.figure.canvas.draw()
            
    
    
    def initPolarPlot(self):
        """
        setup a polar plot canvas
        """

        plt.ion()
        self.figure = plt.figure(figsize=(6, 6), dpi=160, facecolor='w', edgecolor='k')
        self.ax = self.figure.add_subplot(111, polar=True)
        self.lines, = self.ax.plot([],[], linestyle='none', marker='.', markersize=3, markerfacecolor='blue')
        self.ax.set_rmax(5000)
        self.ax.set_theta_direction(-1) #clockwise
        self.ax.set_theta_offset(math.pi/2) #offset by 90 degree so that 0 degree is at 12 o'clock
        #self.ax.grid()
    


    def updatePolarPlot(self):
        """
        re-draw the polar plot with new frameData
        """
        
        if self.frameData.has_new_data:
            _framedata = self.frameData.read_data()
        
            self.lines.set_xdata([_point.angle for _point in list(_framedata)])
            self.lines.set_ydata([_point.distance for _point in list(_framedata) ] )
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
    rplidar.initPolarPlot()
    #rplidar.initXYPlot()
    
    for i in range(100):
        rplidar.readFrameFromQueue()
        rplidar.updatePolarPlot()
        #rplidar.updateXYPlot()
  
        
    rplidar.stopMonitor()
    
    rplidar.disconnect()


    
    
        
            
                    
    rplidar = None
  
