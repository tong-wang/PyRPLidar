"""
RPLidar Types Definition

partly translated from <rptypes.h> of RPLidar SDK v1.4.5
by Tong Wang

 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *

 *
 *  RoboPeak LIDAR System
 *  Common Types definition
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
"""


import logging
import time
from collections import deque
import numpy as np

from rplidar_protocol  import *
from rplidar_cmd import *


class RPLidarRawFrame(object):
    """Raw frame from the RPLidar scan.
    
    Save the timestamp and raw points of a complete frame. This is for archiving
    the data.
    
    Attibutes:
        timestamp: when the frame is initiated and recorded.

        raw_points: a list of points (in raw RPLidar binary format) of a 
        complete frame, starting from the point with syncbit == 1.
    """
    
    def __init__(self):
        self.timestamp = time.time()
        self.raw_points = list()
    
    def add_raw_point(self, raw_point):
        """append new raw_point to the points list."""
        self.raw_points.append(raw_point)


class RPLidarFrame(object):
    """A processed frame with readily usable coordinates.
    
    Contains a moving window (implemented by deques with maxlen) of the most
    recent $maxlen$ points, each being converted from raw point data to both 
    Cartesian and polar coordinates.
    
    This is mainly for real-time visualization of the points.
    
    Attributes:
        angle_d: a deque keeping angle in degrees
        
        angle_r: a deque keeping angle in radians
        
        distance: a deque keeping distance in millimeters
        
        x: a deque keeping x coordinate in millimeters
        
        y: a deque keeping y coordinate in millimeters
        
    Methods:
        add_point(): 
    """
    
    def __init__(self, maxlen=360):

        #self.updated = False
        self.angle_d = deque(maxlen = maxlen)
        self.angle_r = deque(maxlen = maxlen)
        self.distance = deque(maxlen = maxlen)
        self.x = deque(maxlen = maxlen)
        self.y = deque(maxlen = maxlen)
        
    def add_point(self, point):
        """add a parsed point into the deques
        
        Args:
            point: a parsed point in rplidar_response_device_point_format.
        """
        
        angle_d = ((point.angle_highbyte<<7) | point.byte1.angle_lowbyte) / 64.0
        angle_r = np.radians(angle_d)
        distance = point.distance_q2 / 4.0
        
        self.angle_d.append(angle_d)
        self.angle_r.append(angle_r)
        self.distance.append(distance)
        self.x.append(distance * np.sin(angle_r))
        self.y.append(distance * np.cos(angle_r))


class RPLidarPoint(object):
    
    def __init__(self, rawPoint):

        #self.timestamp = time.clock()
        self.raw = rawPoint
        
        parsed = rplidar_response_device_point_format.parse(rawPoint)
        
        self.syncbit = parsed.Byte0.syncbit
        self.syncbit_inverse = parsed.Byte0.syncbit_inverse
        self.quality = parsed.Byte0.sync_quality
        
        self.check_bit = parsed.Byte1.check_bit
        self.angleD = ((parsed.angle_highbyte << 7) | 
                        parsed.Byte1.angle_lowbyte) / 64.0
        self.angle = np.radians(self.angleD)
        
        self.distance = parsed.distance_q2 / 4.0
        
        self.X = self.distance * np.sin(self.angle)
        self.Y = self.distance * np.cos(self.angle)
        
        
# TODO:
#   Complete implementation of all exceptions
#
#

class RPLidarError(Exception):
    def __init__(self, message):
        self.message = message

    def __str__(self):
        return "[RPLidar ERROR] %s\n" % str(self.message)

    def log(self):
        ret = "%s" % str(self.message)
        if(hasattr(self, "reason")):
            return "".join([ret, "\n==> %s" % str(self.reason)])
        return ret



#RESULT_OK = 0
#RESULT_FAIL_BIT = 0x80000000
#RESULT_ALREADY_DONE = 0x20
#RESULT_INVALID_DATA = (0x8000 | RESULT_FAIL_BIT)
#RESULT_OPERATION_FAIL = (0x8001 | RESULT_FAIL_BIT)
#RESULT_OPERATION_TIMEOUT = (0x8002 | RESULT_FAIL_BIT)
#RESULT_OPERATION_STOP = (0x8003 | RESULT_FAIL_BIT)
#RESULT_OPERATION_NOT_SUPPORT = (0x8004 | RESULT_FAIL_BIT)
#RESULT_FORMAT_NOT_SUPPORT = (0x8005 | RESULT_FAIL_BIT)
#RESULT_INSUFFICIENT_MEMORY = (0x8006 | RESULT_FAIL_BIT)

