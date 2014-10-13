"""
RPLidar Archiver

Define classes for a thread that periodically archives raw_frames queue to file.

"""


import logging
import threading
import time
import Queue


class RPLidarArchiver(threading.Thread):
    """ A thread for archiving data from rplidar_monitor
    
        When initiated, the thread will monitor the Queue raw_frames and 
        periodically write the current Queue to file.
    
        Attributes:
            name: thread name.
        
            rplidar: the parent rplidar instance.
        
            alive: the archiver thread is working when alive is set() and stops 
            when alive is clear().
    """
        
    def __init__(self, rplidar):

        logging.debug("Initializing rplidar_archiver thread.")

        threading.Thread.__init__(self)
        self.name = "rplidar_archiver"
        self.rplidar = rplidar
        
        self.alive = threading.Event()
        self.alive.set()
        
        logging.debug("rplidar_archiver thread initialized.")

    
    def save_queue_to_file(self):
        """Write all the content in the raw_frames queue to file. 
        
        File name is automatically set according to the archiving time. Archive 
        file is in an ad-hoc binary format with timestamp, point count of a 
        frame, and all the points in original binary format.
        
        """

        if self.rplidar.raw_frames.qsize() > 0:
            filename = time.strftime("%Y%m%d%H%M") + ".txt"
            with open(filename, 'ab') as fo:
                while not self.rplidar.raw_frames.empty():
                    raw_frame = self.rplidar.raw_frames.get(True, 0.01)
                    fo.write(str(raw_frame.timestamp) + "\n" + 
                             str(len(raw_frame.raw_points)) + "\n" +
                             ''.join(raw_frame.raw_points) + "\n\n");
            logging.debug("Raw_frames archived to file %s", filename)
    
    
    def run(self):

        while self.alive.isSet():
            if time.localtime().tm_sec == 59:
                self.save_queue_to_file()

            time.sleep(1)
    

    def join(self, timeout=None):
        
        self.alive.clear()
        self.save_queue_to_file()
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_archiver thread closed.")
