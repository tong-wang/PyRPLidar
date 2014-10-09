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

        if self.rplidar.raw_frames.qsize() > 0:
            filename = time.strftime("%Y%m%d%H%M") + ".txt"
            with open(filename, 'ab') as fo:
                while True:
                    try: 
                        raw_frame = self.rplidar.raw_frames.get(True, 0.01)
                        fo.write(str(raw_frame.timestamp) + "\n" + 
                                    str(len(raw_frame.raw_points)) + "\n" +
                                    ''.join(raw_frame.raw_points) + "\n\n");
                    except Queue.Empty: 
                        break
            logging.debug("Raw_frames archived to file %s", filename)
    
    
    def run(self):

        self.rplidar.isArchiving = True

        while self.alive.isSet():
            if time.localtime().tm_sec == 59:
                self.save_queue_to_file()

            time.sleep(1)
    
    
    def join(self, timeout=None):
        
        self.alive.clear()
        self.save_queue_to_file()
        self.rplidar.isArchiving = False
        threading.Thread.join(self, timeout)
        logging.debug("rplidar_archiver thread closed.")
