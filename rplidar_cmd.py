"""
RPLidar Commands

partly translated from <rplidar_cmd.h> of RPLidar SDK v1.4.5
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
 *  Data Packet IO packet definition for RP-LIDAR
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
"""


from construct import *


# Commands
# -----------------------------------------

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

RPLIDAR_ANS_TYPE_MEASUREMENT = 0x81
RPLIDAR_ANS_TYPE_DEVINFO = 0x4
RPLIDAR_ANS_TYPE_DEVHEALTH = 0x6

RPLIDAR_STATUS_OK = 0x0
RPLIDAR_STATUS_WARNING = 0x1
RPLIDAR_STATUS_ERROR = 0x2

#RPLIDAR_RESP_MEASUREMENT_SYNCBIT = (0x1<<0)
#RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT = 2
#RPLIDAR_RESP_MEASUREMENT_CHECKBIT = (0x1<<0)
#RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT = 1


# Struct
# ------------------------------------------

# serial data structure returned by GET_INFO (20 bytes)
rplidar_response_device_info_format = Struct("info_format",
    ULInt8("model"),
    ULInt8("firmware_version_minor"),
    ULInt8("firmware_version_major"),
    ULInt8("hardware_version"),
    String("serial_number", 16)
)

# serial data structure returned by GET_HEALTH (3 bytes)
rplidar_response_device_health_format = Struct("health_format",
    Enum(Byte("status"), 
            RPLIDAR_STATUS_OK = RPLIDAR_STATUS_OK,
            RPLIDAR_STATUS_WARNING = RPLIDAR_STATUS_WARNING,
            RPLIDAR_STATUS_ERROR = RPLIDAR_STATUS_ERROR),
    ULInt16("error_code")
)

# serial data structure returned by SCAN -- a single point (5 bytes)
rplidar_response_device_point_format = Struct("point_format",
    BitStruct("byte0", 
                BitField("sync_quality", 6),
                Flag("syncbit_inverse"),
                Flag("syncbit")),
    BitStruct("byte1",
                BitField("angle_lowbyte", 7),
                Const(Flag("check_bit"), 1)), # check_bit must be 1
    ULInt8("angle_highbyte"),
    ULInt16("distance_q2")
)


# convert binary to hex string
toHex = lambda x:"".join([hex(ord(c))[2:].zfill(2) for c in x]).upper()

