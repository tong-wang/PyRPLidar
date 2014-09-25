'''
RPLidar Serial Port Interface

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
 *  Serial based RPlidar Driver
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
'''

import time
from construct import *

from rplidar_protocol  import *
from rplidar_cmd import *
from rplidar_types import *




toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x])


def sendCommand(serial_port, command):
    
    cmdBytes = rplidar_command_format.build(Container(syncByte = RPLIDAR_CMD_SYNC_BYTE, cmd_flag = command))
    serial_port.write(cmdBytes)




def waitResponseHeader(serial_port, timeout=1):
    
    _startTime = time.time()
    
    while time.time() < _startTime + timeout:
        if serial_port.inWaiting() < rplidar_response_header_format.sizeof():
            #print serial_port.inWaiting()
            time.sleep(0.01)
        else:
            _raw = serial_port.read(rplidar_response_header_format.sizeof())
            _parsed = rplidar_response_header_format.parse(_raw)
            #print _parsed
            
            if (_parsed.syncByte1 != RPLIDAR_ANS_SYNC_BYTE1) or (_parsed.syncByte2 != RPLIDAR_ANS_SYNC_BYTE2):
                raise RPLidarError("RESULT_INVALID_ANS_HEADER")
            else:
                return _parsed.type

    raise RPLidarError("RESULT_READING_TIMEOUT")


