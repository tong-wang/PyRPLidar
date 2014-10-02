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

# convert binary to hex string
toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x]).upper()


class RPLidarRawFrame(object):
    
    def __init__(self):
        self.timestamp = time.time()
        self.points = list()
        #self.isComplete = False
    
    def add_point(self, point):
        self.points.append(point)


class RPLidarFrame(object):
    
    def __init__(self):

        #self.updated = False
        self.angle_d = deque(maxlen = 360)
        self.angle_r = deque(maxlen = 360)
        self.distance = deque(maxlen = 360)
        self.x = deque(maxlen = 360)
        self.y = deque(maxlen = 360)
        
    def add_point(self, point):
        
        #self.updated = True
        distance = point.distance_q2 / 4.0
        angle_d = ((point.angle_highbyte<<7) | point.byte1.angle_lowbyte) / 64.0
        angle_r = np.radians(angle_d)
        
        self.angle_d.append(angle_d)
        self.angle_r.append(angle_r)
        self.distance.append(distance)
        self.x.append(distance * np.sin(angle_r))
        self.y.append(distance * np.cos(angle_r))


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

