// GKV_CommunicationLibrary.c: Describes function of GKV_Communication static library.
// WARNING: Library requires GCC C-code compiler!

#include "GKV_CommunicationLibrary.h"
#include <string.h>


const uint32_t crc32_tabl[] =
{
	0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
	0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91, 0x1DB71064,
	0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7, 0x136C9856, 0x646BA8C0,
	0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5, 0x3B6E20C8, 0x4C69105E, 0xD56041E4,
	0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B, 0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940,
	0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59, 0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5,
	0x56B3C423, 0xCFBA9599, 0xB8BDA50F, 0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11,
	0xC1611DAB, 0xB6662D3D, 0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F,
	0x9FBFE4A5, 0xE8B8D433, 0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97,
	0xE6635C01, 0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
	0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65, 0x4DB26158,
	0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB, 0x4369E96A, 0x346ED9FC,
	0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9, 0x5005713C, 0x270241AA, 0xBE0B1010,
	0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F, 0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4,
	0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD, 0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739,
	0x9DD277AF, 0x04DB2615, 0x73DC1683, 0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D,
	0x0A00AE27, 0x7D079EB1, 0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671,
	0x6E6B06E7, 0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
	0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B, 0xD80D2BDA,
	0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79, 0xCB61B38C, 0xBC66831A,
	0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F, 0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92,
	0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D, 0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A,
	0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713, 0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B,
	0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21, 0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B,
	0x6FB077E1, 0x18B74777, 0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD3,
	0x166CCF45, 0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
	0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9, 0xBDBDF21C,
	0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF, 0xB3667A2E, 0xC4614AB8,
	0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/**
  * @name	Configure_Output_Packet
  * @brief  Function inserts selected packet structure into base packet structure, sets values for basic fields and computes crc32.
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet)
  * @param  type - unsigned char value for type of transmitting packet
  * @param  data_ptr - pointer on beginning of packet structure that should be inserted into "data" field of transmitting packet. Packet can be empty
  * @param	size - length of data that should be copied from "data_ptr" into "data" field of transmitting packet
  * @retval no return value.
  */
void Configure_Output_Packet(PacketBase * Output_Packet_Ptr, uint8_t type, void *data_ptr, uint8_t size)
{

	Output_Packet_Ptr->preamble = 0xFF;
	Output_Packet_Ptr->address = 0x01;
	Output_Packet_Ptr->type = type;
	Output_Packet_Ptr->length = size;
	memcpy(Output_Packet_Ptr->data, data_ptr, size);
	*((uint32_t*)&Output_Packet_Ptr->data[size]) = crc32_compute(Output_Packet_Ptr, Output_Packet_Ptr->length + 4);
}


/**
  * @name	Check_Connection
  * @brief  Function configures base packet of 0x00 type and sends it to GKV to check connection. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Check_Connection(GKV_Device *GKV)
{
	uint8_t type = GKV_CHECK_PACKET;
	Configure_Output_Packet(&(GKV->OuputBuffer), type, 0, 0);
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}




/**
  * @name	Request_Data
  * @brief  Function configures base packet of 0x17 type and sends it to GKV to request data packet in "By_Request" GKV data-sending mode. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Request_Data(GKV_Device* GKV)
{
	uint8_t type = GKV_DATA_REQUEST;
	Configure_Output_Packet(&(GKV->OuputBuffer), type, 0, 0);
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}



/**
  * @name	Set_Baudrate
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set new baudrate. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  baudrate_register_value - value for "uart_baud_rate" field of Settings packet structure. List of values - "Baudrate" defgroup
  * @retval no return value.
  */
void Set_Baudrate(GKV_Device* GKV, uint8_t baudrate_register_value)
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	if (baudrate_register_value <= BAUDRATE_3000000)
	{
		GKV_Settings.param_mask |= CHANGE_BAUDRATE;
		GKV_Settings.uart_baud_rate = baudrate_register_value;
	}
	Configure_Output_Packet(&(GKV->OuputBuffer), type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}


/**
  * @name	Set_Default_Algorithm_Packet
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as default packet for current algorithm. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Set_Default_Algorithm_Packet(GKV_Device* GKV)
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	GKV_Settings.mode = SET_DEFAULT_ALGORITHM_PACKET;
	GKV_Settings.mode_mask = ALLOW_CHANGE_SELECTED_PACKET;
	Configure_Output_Packet(&(GKV->OuputBuffer), type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}

/**
  * @name	Set_Custom_Algorithm_Packet
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set sending mode as custom packet for current algorithm. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Set_Custom_Algorithm_Packet(GKV_Device* GKV)
{
	
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;
	GKV_Settings.mode = SET_CUSTOM_PACKET;
	GKV_Settings.mode_mask = ALLOW_CHANGE_SELECTED_PACKET;
	Configure_Output_Packet(&(GKV->OuputBuffer), type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}


/**
  * @name	Set_Algorithm
  * @brief  Function configures base packet of 0x06 type and sends it to GKV to set one of device algorithms. Sending via callback function SendPacketFun
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  algorithm_register_value - value for "algorithm" field of Settings packet structure. List of values - "CURRENT_ALGORITHM" defgroup
  * @retval no return value.
  */
void Set_Algorithm(GKV_Device* GKV, uint8_t algorithm_register_value)
{
	Settings GKV_Settings;
	memset(&GKV_Settings, 0, sizeof(GKV_Settings));
	uint8_t type = GKV_DEV_SETTINGS_PACKET;

	if (algorithm_register_value <= KALMAN_GNSS_NAVIGATON_ALGORITHM)
	{
		GKV_Settings.param_mask |= CHANGE_ALGORITHM;
		GKV_Settings.algorithm = algorithm_register_value;
	}
	Configure_Output_Packet(&(GKV->OuputBuffer), type, &GKV_Settings, sizeof(GKV_Settings));
	Send_Data((GKV->ptrDataSendFunction), &(GKV->OuputBuffer));
}



/**
  * @name	Send_Data
  * @brief  Function run void callback function that sending data to serial interface connected to GKV
  * @param  ptrSendPacketFun - pointer on void-type callback function that gets pointer on PacketBase structure and sends "length" fiels + 8 bytes
  * @param  Output_Packet_Ptr - pointer on beginning of base packet structure that should be transmitted to GKV (pointer on full transmitting packet) 
  * @retval no return value.
  */
void Send_Data(void(*ptrSendPacketFun)(PacketBase* Output_Packet_Ptr), PacketBase* Output_Packet_Ptr)
{
	if (!((ptrSendPacketFun) == NULL))
	{
		ptrSendPacketFun(Output_Packet_Ptr);
	}
}


/*------------------------------Receiving_Data_Parser----------------------------------------------------------------------*/

#define NOT_ENOUGH				0x00
#define REFIND_PREAMBLE			0x01
#define CHECK_OK				0x02


/**
  * @name	GKV_Init_Input
  * @brief  Function initiates with zeros structure that includes input and output GKV Buffers and pointers on packet received callback and sending data function
  * @param  GKV - pointer on selected structure of connected GKV device
  * @retval no return value.
  */
void Init_GKV_Device(GKV_Device* GKV)
{
	GKV->CTR = 0;
	memset(&GKV->InputBuffer, 0, sizeof(GKV->InputBuffer));
	memset(&GKV->OuputBuffer, 0, sizeof(GKV->OuputBuffer));
	GKV->ptrDataSendFunction = NULL;
	GKV->ptrRecognisePacketCallback = NULL;
}



/**
  * @name	put
  * @brief  Function checks current received byte, searching preamble and after finding it puts it into input packet buffer and increments counter
  * @param  b - byte received from UART/COM-port connected to GKV
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching preamble and returns zero until it found.
  */
uint8_t put(uint8_t b, uint32_t *ptr_cnt, PacketBase *buf)//проверка на преамбулу
{
	if (*ptr_cnt == 0)
	{
		if (b != 0xFF)
		{
			return 0;
		}
	}
	
	*((uint8_t *)(buf) + *ptr_cnt) = b;
	(*ptr_cnt)++;
	return 1;
}


/**
  * @name	GKV_ReceiveProcess
  * @brief  Main fuction of received data processing. It can be inserted into main cycle and calls when byte received. function forms packet with received bytes and runs callback fucntion when it formed.
  * @param  GKV - pointer on selected structure of connected GKV device
  * @param  ReceivedByte - current received byte from selected GKV device
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t GKV_ReceiveProcess(GKV_Device * GKV, char ReceivedByte)
{
	uint32_t * ptr_cnt = (uint32_t *)&(GKV->CTR);
	PacketBase* ptr_buf = (PacketBase*)&(GKV->InputBuffer);
	char b = ReceivedByte;

	if (put(b, ptr_cnt, ptr_buf))
	{
		return parseCycle(GKV->ptrRecognisePacketCallback, ptr_cnt, ptr_buf);
	}
	return 0;
}




/**
  * @name	parseCycle
  * @brief  Parcing cycle function. When new byte added to input packet buffer. Function checks number of received bytes and checksum result.
  * @param  ptrRecognisePacket - pointer on user callback function that should process packet when correct packet receive
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t parseCycle(void (*ptrRecognisePacket)(PacketBase* buf), uint32_t* ptr_cnt, PacketBase* buf)
{
	uint8_t status = 0;
	while (1)
	{
		status = parse(ptr_cnt, buf);
		if (status == NOT_ENOUGH)
		{
			break;
		}
		else if (status == REFIND_PREAMBLE)
		{
			if (!refind_preamble(1, ptr_cnt, buf))
				break;
		}
		else if (status == CHECK_OK)
		{
			if (!((ptrRecognisePacket) == NULL))
			{
				ptrRecognisePacket(buf);
			}
			if (!refind_preamble((buf->length) + 8, ptr_cnt, buf))
				break;
		}
	}
	if (status < CHECK_OK)
	{
		status = 0;
	}
	return status;
}



/**
  * @name	refind_preamble
  * @brief  Moving memory function when checksum is incorrect to check next 0xFF as preamble or moving memory when correct packet processed. 
  * @param  start - byte with number of start byte to move memory when checksum is incorrect (1) or when received packet is correct (buf->length) + 8 .
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval Function returns 1 when 0xFF found after memory moving and 0, when it wasn't found.
  */
uint8_t refind_preamble(int start, uint32_t *ptr_cnt, PacketBase *buf)
{
	uint8_t * in_buf = (uint8_t*)(buf);
	for (int i = start; i < *ptr_cnt; i++)
	{
		if (*(in_buf +i) == 0xFF)
		{
			*ptr_cnt -= i;
			memmove(in_buf, (in_buf + i), *ptr_cnt);
			return 1;
		}
	}
	*ptr_cnt = 0;
	return 0;
}



/**
  * @name	parse
  * @brief  Parcing step function. Trying to find correct packet in current number of received bytes 
  * @param  ptr_cnt - pointer on byte counter in input packet buffer
  * @param  buf - pointer on beginning of input packet buffer
  * @retval function returns result of searching correct packet. 0x00 - not enough bytes received, 0x01 - checksum is incorrect, 0x02 - packet checked
  */
uint8_t parse(uint32_t *ptr_cnt, PacketBase *buf)
{
	if (*ptr_cnt >= 4)
	{
		if (buf->length > DATA_LENGTH)
		{
			return REFIND_PREAMBLE;
		}
	}

	if (*ptr_cnt < (buf->length)+8)
	{
		return NOT_ENOUGH;
	}
	if (!check(buf))
	{
		return REFIND_PREAMBLE;
	}

	return CHECK_OK;
}



/**
  * @name	crc32_compute
  * @brief  CRC32 checksum calculation
  * @param  buf - pointer on beginning of packet
  * @param  size - length of packet without checksum
  * @retval function returns result crc32 calculation.
  */
uint32_t crc32_compute(const void* buf, unsigned long size)
{
	uint32_t crc = 0;
	const uint8_t* p = (const uint8_t*)buf;
	crc = crc ^ 0xFFFFFFFFUL;

	while (size--)
		crc = crc32_tabl[(crc ^ *p++) & 0xFF] ^ (crc >> 8);

	return crc ^ 0xFFFFFFFFUL;
}



/**
  * @name	check
  * @brief  chcecking input packet buffer for crc32
  * @param  pack - pointer on beginning of input packet buffer
  * @retval function returns result of checking crc32. 0x00 - checksum is incorrect, 0x01 - checksum is correct
  */
uint8_t check(PacketBase* pack)
{
	if (pack->preamble != 0xFF)
		return false;
	if (pack->length > 255)
		return false;

	uint32_t crc = crc32_compute(pack, pack->length + 4);
	uint8_t* p = &pack->data[pack->length];
	uint8_t* t = (uint8_t*)&crc;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;
	if (*p++ != *t++) return false;

	return true;
	return false;


}