#pragma once
/**
  ******************************************************************************
  * @file    GKV_CommunicationLibrary.h
  * @author  Alex Galkin,  <info@mp-lab.ru>
  * @brief   List of default Request_Response structures for inertial navigation module GKV-10 and functions for general requests and parcing input data
  *  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Laboratory of Microdevices, Ltd. </center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Laboratory of Microdevices nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __GKV_COMMUNICATION_LIBRARY_H__
#define __GKV_COMMUNICATION_LIBRARY_H__

#include "GKV_BasePacket.h"
#include "GKV_TestPacket.h"
#include "GKV_SettingsPacket.h"
#include "GKV_AlorithmPackets.h"
#include "GKV_CustomPacket.h"
#include "GKV_FilterPacket.h"
#include "GKV_SelfTestPacket.h"
#include "GKV_AlgParamPacket.h"
#include "GKV_GyroOffsetPacket.h"


#define true 1
#define false 0

typedef struct __GKV_Device
{
    void (*ptrRecognisePacketCallback)(PacketBase* Output_Packet_Ptr);
    void (*ptrDataSendFunction)(PacketBase* Output_Packet_Ptr);
    PacketBase InputBuffer;
    PacketBase OuputBuffer;
    uint32_t CTR;
}GKV_Device;



/**
  * @name	Configure_Output_Packet
  * @brief  Function inserts selected packet structure into base packet structure, sets values for basic fields and computes crc32.
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet)
  * @param  type - unsigned char value for type of transmitting packet
  * @param  data_ptr - pointer on beginning of packet structure that should be inserted into "data" field of transmitting packet. Packet can be empty
  * @param	size - length of data that should be copied from "data_ptr" into "data" field of transmitting packet
  * @retval no return value.
  */
void Configure_Output_Packet(PacketBase * Output_Packet, uint8_t type, void * data_ptr, uint8_t size);


/**
  * @name	Check_Connection
  * @brief  Function configures base packet of 0x00 type and sends it to GKV to check connection. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Check_Connection(GKV_Device* GKV);


/**
  * @name	Request_Data
  * @brief  Function configures base packet of 0x17 type and sends it to GKV to request data packet in "By_Request" GKV data-sending mode. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Request_Data(GKV_Device* GKV);


/**
  * @name	Set_Baudrate
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set new baudrate. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  baudrate_register_value - value for "uart_baud_rate" field of Settings packet structure. List of values - "Baudrate" defgroup
  * @retval no return value.
  */
void Set_Baudrate(GKV_Device* GKV, uint8_t baudrate_register_value);




/**
  * @name	Set_Default_Algorithm_Packet
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as default packet for current algorithm. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Set_Default_Algorithm_Packet(GKV_Device* GKV);




/**
  * @name	Set_Custom_Algorithm_Packet
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as custom packet for current algorithm. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Set_Custom_Algorithm_Packet(GKV_Device* GKV);





/**
  * @name	Set_Algorithm
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set one of device algorithms. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  algorithm_register_value - value for "algorithm" field of Settings packet structure. List of values - "CURRENT_ALGORITHM" defgroup
  * @retval no return value.
  */
void Set_Algorithm(GKV_Device* GKV, uint8_t algorithm_register_value);

/**
  * @name	Send_Data
  * @brief  Function run void callback function that sending data to serial interface connected to GKV
  * @param  ptrSendPacketFun - pointer on void-type callback function that gets pointer on PacketBase structure and sends "length" fiels + 8 bytes
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet)
  * @retval no return value.
  */
void Send_Data(void(*ptrSendPacketFun)(PacketBase* Output_Packet_Ptr), PacketBase* Output_Packet_Ptr);

/*---------------------------------------------------------Section with function prototypes for receive data parser-------------------------------------------------------------------------*/


/**
  * @name	GKV_Init_Input
  * @brief  Function initiates with zeros structure that includes input and output GKV Buffers and pointers on packet received callback and sending data function
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Init_GKV_Device(GKV_Device* GKV);


/**
  * @name	put
  * @brief  Function checks current received byte, searching preamble and after finding it puts it into input packet buffer and increments counter
  * @param  b - byte received from UART/COM-port connected to GKV
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching preamble and returns zero until it found.
  */
uint8_t put(uint8_t b, uint32_t* ptr_cnt, PacketBase* buf);



/**
  * @name	GKV_ReceiveProcess
  * @brief  Main fuction of received data processing. It can be inserted into main cycle and calls when byte received. function forms packet with received bytes and runs callback fucntion when it formed.
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  ReceivedByte - current received byte from selected GKV device
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t GKV_ReceiveProcess(GKV_Device* GKV, char ReceivedByte);



/**
  * @name	parseCycle
  * @brief  Parcing cycle function. When new byte added to input packet buffer. Function checks number of received bytes and checksum result.
  * @param  ptrRecognisePacket - pointer on user callback function that should process packet when correct packet receive
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t parseCycle(void (*ptrRecognisePacket)(PacketBase* buf), uint32_t* ptr_cnt, PacketBase* buf); /*???????????????????*/



/**
  * @name	parse
  * @brief  Parcing step function. Trying to find correct packet in current number of received bytes
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t parse(uint32_t* ptr_cnt, PacketBase* buf);



/**
  * @name	refind_preamble
  * @brief  Moving memory function when checksum is incorrect to check next 0xFF as preamble or moving memory when correct packet processed.
  * @param  start - byte with number of start byte to move memory when checksum is incorrect (1) or when received packet is correct (buf->length) + 8 .
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval Function returns 1 when 0xFF found after memory moving and 0, when it wasn't found.
  */
uint8_t refind_preamble(int start, uint32_t* ptr_cnt, PacketBase* buf);


/**
  * @name	crc32_compute
  * @brief  CRC32 checksum calculation
  * @param  buf - pointer on beginning of packet  
  * @param  size - length of packet without checksum
  * @retval function returns result crc32 calculation.
  */
uint32_t crc32_compute(const void* buf, unsigned long size);

/**
  * @name	check
  * @brief  chcecking input packet buffer for crc32
  * @param  pack - pointer on beginning of input packet buffer
  * @retval function returns result of checking crc32. 0x00 - checksum is incorrect, 0x01 - checksum is correct
  */
uint8_t check(PacketBase* pack);

#endif