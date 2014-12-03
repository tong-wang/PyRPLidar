""" 
RPLidar Python Driver

...
...

"""


import serial
import logging
import Queue
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

from rplidar_monitor import *


class RPLidar(object):
    
    def __init__(self, portname, baudrate=115200, timeout=1):
        
        # init serial port
        self.serial_port = None
        self.portname = portname
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_arg = dict(port=portname,
                               baudrate=baudrate,
                               stopbits=serial.STOPBITS_ONE,
                               parity=serial.PARITY_NONE,
                               timeout=timeout)
        
        # status variables
        self.isConnected = False
        self.motorRunning = None
        
        # init monitor
        self.monitor = None
        
        # data containers
        self.raw_points = Queue.Queue()
        self.raw_frames = Queue.Queue()
        self.current_frame = RPLidarFrame()

        
    def connect(self):

        if not self.isConnected:
            try:
                self.serial_port = serial.Serial(**self.serial_arg)
                self.isConnected = True
                logging.debug("Connected to RPLidar on port %s", self.portname)
                self.stop_motor()
            except serial.SerialException as e:
                logging.error(e.message)
               

    def disconnect(self):

        if self.isConnected:
            try:
                if self.monitor:
                    self.stop_monitor()
                self.serial_port.close()
                self.isConnected = False
                logging.debug("Disconnected from RPLidar on port %s", self.portname)
            except serial.SerialException as e:
                logging.error(e.message)


    def reset(self):

        self.send_command(RPLIDAR_CMD_RESET)
        logging.debug("Command RESET sent.")
        time.sleep(0.1)
        

    def start_motor(self):
        """Start RPLidar motor by setting DTR (which is connected to pin MOTOCTL
        on RPLidar) to False."""

        self.serial_port.setDTR(False)
        self.motorRunning = True
        logging.debug("RPLidar motor is turned ON.")


    def stop_motor(self):
        """Stop RPLidar motor by setting DTR to True."""

        self.serial_port.setDTR(True)
        self.motorRunning = False
        logging.debug("RPLidar motor is turned OFF.")


    def send_command(self, command):
        """Send command to RPLidar through the serial connection"""
        
        cmd_bytes = rplidar_command_format.build(Container(
                             sync_byte=RPLIDAR_CMD_SYNC_BYTE, cmd_flag=command))
        self.serial_port.write(cmd_bytes)
        logging.debug("Command %s sent.", toHex(cmd_bytes))
    
    
    def response_header(self, timeout=1):
        """Read response header from RPLidar through the serial connection"""
        
        start_time = time.time()
        
        while time.time() < start_time + timeout:
            if self.serial_port.inWaiting() < rplidar_response_header_format.sizeof():
                #logging.debug(serial_port.inWaiting())
                time.sleep(0.01)
            else:
                raw = self.serial_port.read(rplidar_response_header_format.sizeof())
                parsed = rplidar_response_header_format.parse(raw)
                #logging.debug(parsed)
                
                if ((parsed.sync_byte1 != RPLIDAR_ANS_SYNC_BYTE1) or 
                    (parsed.sync_byte2 != RPLIDAR_ANS_SYNC_BYTE2)):
                    raise RPLidarError("RESULT_INVALID_ANS_HEADER")
                else:
                    return parsed.response_type
    
        raise RPLidarError("RESULT_READING_TIMEOUT")
    
    
    def get_device_info(self):
        """Obtain hardware information about RPLidar"""
        
        self.serial_port.flushInput()
        
        self.send_command(RPLIDAR_CMD_GET_DEVICE_INFO)

        if self.response_header() == RPLIDAR_ANS_TYPE_DEVINFO:
            raw = self.serial_port.read(rplidar_response_device_info_format.sizeof())
            parsed = rplidar_response_device_info_format.parse(raw)
        
            return {"model": parsed.model, 
                    "firmware_version_major": parsed.firmware_version_major, 
                    "firmware_version_minor": parsed.firmware_version_minor, 
                    "hardware_version": parsed.hardware_version,
                    "serial_number": toHex(parsed.serial_number)}

        else:
            raise RPLidarError("RESULT_INVALID_ANS_TYPE")


    def get_health(self):
        """Obtain health information about RPLidar"""

        self.serial_port.flushInput()
        
        self.send_command(RPLIDAR_CMD_GET_DEVICE_HEALTH)

        if self.response_header() == RPLIDAR_ANS_TYPE_DEVHEALTH:
            raw = self.serial_port.read(rplidar_response_device_health_format.sizeof())
            parsed = rplidar_response_device_health_format.parse(raw)
                
            return {"status": parsed.status, 
                    "error_code": parsed.error_code}
        else:
            raise RPLidarError("RESULT_INVALID_ANS_TYPE")


    def start_monitor(self, archive=False):
        """ Start the monitor thread """
        
        if self.monitor is None:
            logging.debug("Try to start monitor thread.")
            self.monitor = RPLidarMonitor(self, archive=archive)
            self.monitor.start()


    def stop_monitor(self):
        """ Stop the monitor """

        if self.monitor is not None:
            logging.debug("Try to stop monitor thread.")
            self.monitor.join()
            self.monitor = None


    def init_xy_plot(self):
        """ setup an XY plot canvas """
        
        plt.ion()
        self.figure = plt.figure(figsize=(6, 6),
                                 dpi=160,
                                 facecolor="w",
                                 edgecolor="k")
        self.ax = self.figure.add_subplot(111)
        self.lines, = self.ax.plot([],[],
                                   linestyle="none",
                                   marker=".",
                                   markersize=3,
                                   markerfacecolor="blue")
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.grid()
    
    
    def update_xy_plot(self):
        """ re-draw the XY plot with new current_frame """

        self.lines.set_xdata(self.current_frame.x)
        self.lines.set_ydata(self.current_frame.y)
        self.figure.canvas.draw()
    
    
    def init_polar_plot(self):
        """ setup a polar plot canvas """

        plt.ion()
        self.figure = plt.figure(figsize=(6, 6), 
                                 dpi=160, 
                                 facecolor="w", 
                                 edgecolor="k")
        self.ax = self.figure.add_subplot(111, polar=True)
        self.lines, = self.ax.plot([],[], 
                                   linestyle="none", 
                                   marker=".", 
                                   markersize=3, 
                                   markerfacecolor="blue")
        self.ax.set_rmax(5000)
        self.ax.set_theta_direction(-1) #set to clockwise
        self.ax.set_theta_offset(np.pi/2) #offset by 90 degree so that 0 degree is at 12 o'clock
        #self.ax.grid()


    def update_polar_plot(self):
        """ re-draw the polar plot with new current_frame """

        self.lines.set_xdata(self.current_frame.angle_r)
        self.lines.set_ydata(self.current_frame.distance)
        self.figure.canvas.draw()
       
        
        
        
        


if __name__ == "__main__":
    
    # logging config
    logging.basicConfig(level=logging.DEBUG,
                    format="[%(levelname)s] (%(threadName)-10s) %(message)s")

    rplidar = RPLidar("/dev/tty.SLAB_USBtoUART")
    rplidar.connect()
   
    print rplidar.get_device_info()
    print rplidar.get_health()


    rplidar.start_monitor(archive=True)
    
    #rplidar.init_polar_plot()
    rplidar.init_xy_plot()
    
    try:
        while True:
            #rplidar.update_polar_plot()
            rplidar.update_xy_plot()
            time.sleep(0.15)
    except KeyboardInterrupt:
        logging.debug("CTRL-c pressed, exiting...")
        pass

    rplidar.stop_monitor()
    rplidar.disconnect()
    rplidar = None
