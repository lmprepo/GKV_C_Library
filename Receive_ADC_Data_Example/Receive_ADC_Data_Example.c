
#include <windows.h>
#include <stdio.h>
#include <GKV_CommunicationLibrary.h>

HANDLE hSerial;

char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);


int main()
{
    char com_port[] = "COM9";

    printf("Set Serial Port:%s\n", com_port);

    printf ("#start connecting to %s\n", com_port);

    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm = ADC_CODES_ALGORITHM;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    uint8_t algorithm_selected = 0;

    InitInput InputStructure;
    GKV_Init_Input(&InputStructure);

    hSerial = CreateFile(com_port, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        if (GetLastError() == ERROR_FILE_NOT_FOUND)
        {
            printf("serial port does not exist.\n");
            return 1;
        }
        printf("some other error occurred.\n");
        return 1;
    }
    printf("#connect ok\n");
    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        printf("getting state error\n");
        return 1;
    }
    printf("#get state ok\n");
    dcbSerialParams.BaudRate = 921600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        printf("error setting serial port state\n");
        return 1;
    }
    printf("#set state ok, waiting for data...\n");

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
    while (1)
    {
        InputStructure.GKV_Byte = ReadCOM();
        GKV_Process(RecognisePacket, &InputStructure);
    }


    return 0;
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
    case GKV_RAW_DATA_PACKET:
    {
        RawData* packet;
        packet = (RawData*)&buf->data;
        sprintf(str, "%d", packet->sample_cnt);
        printf("Sample Counter = %s", str);

        sprintf(str, "%f", packet->a[0]);
        printf(" ax = %s", str);

        sprintf(str, "%f", packet->a[1]);
        printf(" ay = %s", str);

        sprintf(str, "%f", packet->a[2]);
        printf(" az = %s", str);

        sprintf(str, "%f", packet->w[0]);
        printf(" wx = %s", str);

        sprintf(str, "%f", packet->w[1]);
        printf(" wy = %s", str);

        sprintf(str, "%f", packet->w[2]);
        printf(" wz = %s\n", str);
        break;
    }
    }
}