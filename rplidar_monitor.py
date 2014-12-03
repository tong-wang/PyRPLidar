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
from rplidar_archiver import *


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

            archive: the flag indicating whether to archive data.

            archiver: the archiver instance.
    """


    def __init__(self, rplidar, archive=False):

        logging.debug("Initializing rplidar_monitor thread.")

        threading.Thread.__init__(self)
        self.name = "rplidar_monitor"
        self.rplidar = rplidar
        self.alive = threading.Event()
        self.alive.set()

        # init archiver
        self.archive = archive
        self.archiver = None

        logging.debug("rplidar_monitor thread initialized.")


    def start_scan(self):
        """Send SCAN command to RPLidar."""

        self.rplidar.start_motor()
        self.rplidar.send_command(RPLIDAR_CMD_SCAN)

        if self.rplidar.response_header() != RPLIDAR_ANS_TYPE_MEASUREMENT:
            raise RPLidarError("RESULT_INVALID_MEASUREMENT_HEADER")

        logging.debug("Start scanning.")

        if self.archive:
            self.start_archiver()


    def stop_scan(self):
        """Send STOP command to RPLidar."""

        self.rplidar.stop_motor()
        self.rplidar.send_command(RPLIDAR_CMD_STOP)

        # it seems to be very important to have a pause here
        # until the STOP command is executed by RPLidar
        time.sleep(0.1)

        logging.debug("Stop scanning.")

        if self.archive:
            self.stop_archiver()


    def start_archiver(self):
        """ Start the archiver thread """

        if self.archiver is None:
            logging.debug("Try to start archiver thread.")
            self.archiver = RPLidarArchiver(self.rplidar)
            self.archiver.start()


    def stop_archiver(self):
        """ Stop the archiver thread """

        if self.archiver is not None:
            logging.debug("Try to stop archiver thread.")
            self.archiver.join()
            self.archiver = None


    def run(self):
        """Main thread function.

        Continuously reads raw data feed (raw_point) from RPLidar and
        modifies:
            raw_points: raw_point is directly put into it;

            raw_frames: everytime when there is a point with syncbit==True, the
            previous raw_frame is put into raw_frames, a new raw_frame instance
            is initiated. The raw_point is added into raw_frame.

            current_frame: raw_point is first parsed then added into it;
        """

        self.start_scan()

        raw_frame = None

        while self.alive.isSet():
            while self.rplidar.serial_port.inWaiting() < 5:
                time.sleep(.0001)

            raw_point = self.rplidar.serial_port.read(5)
            point = rplidar_response_device_point_format.parse(raw_point)

            # save to raw_points
            self.rplidar.raw_points.put(raw_point)

            # save to raw_frames:
            # when syncbit == True, meaning a new frame starts,
            # the existing frame, if any, is saved in raw_frames
            # then initialize a new empty frame
            if point.byte0.syncbit:
                if raw_frame:
                    self.rplidar.raw_frames.put(raw_frame)
                    logging.debug("raw_frames qsize: %d, raw_frame length: %d.",
                     self.rplidar.raw_frames.qsize(), len(raw_frame.raw_points))

                raw_frame = RPLidarRawFrame()

            raw_frame.add_raw_point(raw_point)

            # save to current_frame
            self.rplidar.current_frame.add_point(point)


    def join(self, timeout=0.1):

        self.alive.clear()
        self.stop_scan()
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_monitor thread closed.")
