'''
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
 *
 *  RoboPeak LIDAR System
 *  Data Packet IO packet definition for RP-LIDAR
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
'''


import time
from construct import *


# Commands
#-----------------------------------------
RPLIDAR_CMD_SYNC_BYTE = 0xa5
RPLIDAR_CMDFLAG_HAS_PAYLOAD = 0x80

# Commands without payload and response
RPLIDAR_CMD_STOP = 0x25
RPLIDAR_CMD_SCAN = 0x20
RPLIDAR_CMD_FORCE_SCAN = 0x21
RPLIDAR_CMD_RESET = 0x40


# Commands without payload but have response
RPLIDAR_CMD_GET_DEVICE_INFO = 0x50
RPLIDAR_CMD_GET_DEVICE_HEALTH = 0x52


# Response
# ------------------------------------------
RPLIDAR_ANS_SYNC_BYTE1 = 0xa5
RPLIDAR_ANS_SYNC_BYTE2 = 0x5a

RPLIDAR_ANS_TYPE_MEASUREMENT = 0x81
RPLIDAR_ANS_TYPE_DEVINFO = 0x04
RPLIDAR_ANS_TYPE_DEVHEALTH = 0x06



# Struct
# ------------------------------------------
#serial data structure for CMD header (2 bytes)
rplidar_command_format = Struct('cmd_format',
    ULInt8('syncByte'), # A5
    ULInt8('cmd'), # 1 byte for CMD
)


#serial data structure for response header (7 bytes)
rplidar_response_header_format = Struct('header_format',
    ULInt8('syncByte1'),#, 1), # A5
    ULInt8('syncByte2'),#, 1), # 5A
    ULInt32('black'),#, 4), # 4 bytes for unknown purpose
    ULInt8('type')#, 1), # 1 bytes for message type
)

#serial data structure returned by GET_INFO (20 bytes)
rplidar_response_device_info_format = Struct('info_format',
    ULInt8('model'),
    ULInt8('firmware_minor'),
    ULInt8('firmware_major'),
    ULInt8('hardware'),
    String('serialno', 16)
)


#serial data structure returned by GET_HEALTH (3 bytes)
rplidar_response_device_health_format = Struct('health_format',
    Enum(Byte('status'), RPLIDAR_STATUS_OK = 0, RPLIDAR_STATUS_WARNING = 1, RPLIDAR_STATUS_ERROR = 2, RPLIDAR_STATUS_ERROR_UNKNOWN = 3),
    ULInt16('error_code')
)


#serial data structure returned by SCAN -- a single point (5 bytes)
rplidar_response_device_point_format = Struct("point_format",
    BitStruct("Byte0", BitField("quality", 6), Flag("S_bar"), Flag("S")),
    BitStruct("Byte1", BitField("angleL", 7), Flag("C")), # =1
    ULInt8("angleH"),
    ULInt16("distance_q2"),
    Value("distance", lambda ctx: ctx["distance_q2"]/4.0),
    Value("angle", lambda ctx: ((ctx["angleH"] << 7) | ctx["Byte1"].angleL)/64.0)
)
    



def sendCommand(serial_port, command):
    
    cmdBytes = rplidar_command_format.build(Container(syncByte = RPLIDAR_CMD_SYNC_BYTE, cmd = command))
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



RESULT_OK = 0
RESULT_FAIL_BIT = 0x80000000
RESULT_ALREADY_DONE = 0x20
RESULT_INVALID_DATA = (0x8000 | RESULT_FAIL_BIT)
RESULT_OPERATION_FAIL = (0x8001 | RESULT_FAIL_BIT)
RESULT_OPERATION_TIMEOUT = (0x8002 | RESULT_FAIL_BIT)
RESULT_OPERATION_STOP = (0x8003 | RESULT_FAIL_BIT)
RESULT_OPERATION_NOT_SUPPORT = (0x8004 | RESULT_FAIL_BIT)
RESULT_FORMAT_NOT_SUPPORT = (0x8005 | RESULT_FAIL_BIT)
RESULT_INSUFFICIENT_MEMORY = (0x8006 | RESULT_FAIL_BIT)

IS_OK = lambda x: ( ((x) & RESULT_FAIL_BIT) == 0 )
IS_FAIL = lambda x: ( ((x) & RESULT_FAIL_BIT) )