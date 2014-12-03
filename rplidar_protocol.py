"""
RPLidar Protocol

partly translated from <rplidar_protocol.h> of RPLidar SDK v1.4.5
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
 *  Data Packet IO protocol definition for RP-LIDAR
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  
"""


from construct import *


# Protocol
# -----------------------------------------

RPLIDAR_CMD_SYNC_BYTE = 0xA5
RPLIDAR_CMDFLAG_HAS_PAYLOAD = 0x80

RPLIDAR_ANS_SYNC_BYTE1 = 0xA5
RPLIDAR_ANS_SYNC_BYTE2 = 0x5A

#RPLIDAR_ANS_PKTFLAG_LOOP = 0x1

#RPLIDAR_ANS_HEADER_SIZE_MASK = 0x3FFFFFFF
#RPLIDAR_ANS_HEADER_SUBTYPE_SHIFT = 30


# Struct
# ------------------------------------------

# serial data structure for CMD header (2 bytes)
rplidar_command_format = Struct("cmd_format",
    ULInt8("sync_byte"), # must be RPLIDAR_CMD_SYNC_BYTE: A5
    ULInt8("cmd_flag") # one byte for CMD
)

# serial data structure for response header (7 bytes)
rplidar_response_header_format = Struct("header_format",
    ULInt8("sync_byte1"), # must be RPLIDAR_ANS_SYNC_BYTE1: A5
    ULInt8("sync_byte2"), # must be RPLIDAR_ANS_SYNC_BYTE2: 5A
    #ULInt32("size_q30_subtype"), # see _u32 size:30; _u32 subType:2;
    ULInt24("response_size"),
    BitStruct("response_mode", 
            Enum(BitField("mode", 2), SINGLE = 0x0, MULTI = 0x1, _default_ = "UNDEFINED"),
            Padding(6)),
    ULInt8("response_type") # one byte for message type
)

