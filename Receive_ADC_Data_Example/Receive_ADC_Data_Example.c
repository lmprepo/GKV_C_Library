#include <windows.h>
#include <stdio.h>
#include <GKV_CommunicationLibrary.h>

HANDLE hSerial;

uint8_t InitSerialPort(char* port_name, int32_t baudrate);
char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);
char* cin(int* length);

int main()
{
   
    int length = 0;
    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm = ADC_CODES_ALGORITHM;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    uint8_t algorithm_selected = 0;
    /* Select serial port*/
    printf("Set Serial Port:");
    char* com_port = cin(&length);
    /* Init GKV Receive Data Structure */
    InitInput InputStructure;
    GKV_Init_Input(&InputStructure);
    /* Connect to Selected serial port*/
    printf("#start connecting to %s\n", com_port);
    if (!(InitSerialPort(com_port, 921600))) return 1;
    /* Waiting for device connection and selecting algorithm */
    const int MAX_INCORRECT_CNT = 1000;
    int incorrectCnt = 0;
    while (!(algorithm_selected))
    {
        Set_Algorithm(WriteCOM, algorithm);
        Packet_is_Correct = 0;
        while (!(Packet_is_Correct))
        {
            InputStructure.GKV_Byte = ReadCOM();
            Packet_is_Correct = GKV_Process(RecognisePacket, &InputStructure);
            incorrectCnt++;
            if (incorrectCnt > MAX_INCORRECT_CNT)
            {
                printf( "error too many incorrect packets\n");
                return 1;
            }
        }
        if (InputStructure.InputPacket.type == algorithm_packet)
        {
            algorithm_selected = 1;
        }
    }
    printf("#start read loop\n");
    /* Receive Data for Selected algorithm */
    while (1)
    {
        InputStructure.GKV_Byte = ReadCOM();
        GKV_Process(RecognisePacket, &InputStructure);
    }
    return 0;
}


uint8_t InitSerialPort(char* port_name, int32_t baudrate)
{
    hSerial = CreateFile(port_name, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            printf("serial port does not exist.\n");
            return 0;
        }
        printf("some other error occurred.\n");
        return 0;
    }
    printf("#connect ok\n");
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        printf("getting state error\n");
        return 0;
    }
    printf("#get state ok\n");
    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        printf("error setting serial port state\n");
        return 0;
    }
    printf("#set state ok, waiting for data...\n");
    return 1;
}

char* cin(int* length) {
    *length = 0;
    int capacity = 1;
    char* str = (char*)malloc(sizeof(char));
    char sym = getchar();
    while (sym != '\n') {
        str[(*length)++] = sym;
        if (*length >= capacity) {
            capacity *= 2;
            str = (char*)realloc(str, capacity * sizeof(char));
        }
        sym = getchar();
    }
    str[*length] = '\0';
    return str;
}

void WriteCOM(PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
}

char ReadCOM()
{
    DWORD iSize;
    char sReceivedChar;
    while (true)
    {
        ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
        if (iSize > 0)
            return sReceivedChar;
    }
}

/* User Callback on any Received Packet */

void RecognisePacket(PacketBase* buf)
{
    char str[30];

    switch (buf->type)
    {
        case GKV_ADC_CODES_PACKET:
        {
            ADCData* packet;
            packet = (ADCData*)&buf->data;
            sprintf(str, "%d", packet->sample_cnt);
            printf("Sample Counter = %s", str);

            sprintf(str, "%d", packet->a[0]);
            printf(" ax = %s", str);

            sprintf(str, "%d", packet->a[1]);
            printf(" ay = %s", str);

            sprintf(str, "%d", packet->a[2]);
            printf(" az = %s", str);

            sprintf(str, "%d", packet->w[0]);
            printf(" wx = %s", str);

            sprintf(str, "%d", packet->w[1]);
            printf(" wy = %s", str);

            sprintf(str, "%d", packet->w[2]);
            printf(" wz = %s\n", str);
            break;
        }
        case GKV_GNSS_PACKET:
        {
            GpsData* packet;
            packet = (GpsData*)&buf->data;
            printf(" GNSS Data Packet: ");
            sprintf(str, "%f", packet->time);
            printf(" time = %s", str);
            sprintf(str, "%f", packet->latitude);
            printf(" latitude = %s", str);
            sprintf(str, "%f", packet->longitude);
            printf(" longitude = %s", str);
            sprintf(str, "%f", packet->altitude);
            printf(" altitude = %s", str);
            sprintf(str, "%d", packet->state_status);
            printf(" state_status = %s", str);
            sprintf(str, "%f", packet->TDOP);
            printf(" TDOP = %s", str);
            sprintf(str, "%f", packet->HDOP);
            printf(" HDOP = %s", str);
            sprintf(str, "%f", packet->VDOP);
            printf(" VDOP = %s\n", str);
            break;
        }
    }
}