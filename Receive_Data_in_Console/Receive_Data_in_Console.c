﻿#ifdef _WIN32
#include <windows.h>
#include <stdio.h>
#include <GKV_CommunicationLibrary.h>

HANDLE hSerial;

uint8_t InitSerialPort(char* port_name, int32_t baudrate);
char ReadCOM();
void WriteCOM(GKV_PacketBase* buf);
void RecognisePacket(GKV_PacketBase* buf);
char* cin(int* length);

int main()
{
    int length = 0;
    /* Select serial port*/
    printf("Set Serial Port:");
    char* com_port = cin(&length);
    /* Init GKV Receive Data Structure */
    GKV_Device GKV;
    Init_GKV_Device(&GKV);
    GKV.ptrRecognisePacketCallback = RecognisePacket;
    GKV.ptrDataSendFunction = WriteCOM;
    /* Connect to Selected serial port*/
    printf("#start connecting to %s\n", com_port);
    if (!(InitSerialPort(com_port, 921600))) return 1;
    /* Read Data From GKV in Main Cycle */
    while (1)
    {
        GKV_ReceiveProcess(&GKV, ReadCOM());
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

void WriteCOM(GKV_PacketBase* buf)
{
    DWORD dwBytesWritten;
    char iRet = WriteFile(hSerial, buf, buf->length + 8, &dwBytesWritten, NULL);
    Sleep(1);
}

char ReadCOM()
{
    DWORD iSize;
    char sReceivedChar;
    char iRet = 0;
    while (true)
    {
        iRet = ReadFile(hSerial, &sReceivedChar, 1, &iSize, 0);
        if (iSize > 0)
            return sReceivedChar;
    }
}

/* User Callback on any Received Packet */


void RecognisePacket(GKV_PacketBase* buf)
{
    char str[30];

    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        GKV_ADCData* packet;
        packet = (GKV_ADCData*)&buf->data;
        printf("ADC Data Packet: ");
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
        GKV_RawData* packet;
        packet = (GKV_RawData*)&buf->data;
        printf("Raw Sensors Data Packet: ");
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
    case GKV_EULER_ANGLES_PACKET:
    {
        GKV_GyrovertData* packet;
        packet = (GKV_GyrovertData*)&buf->data;
        printf(" Gyrovert Data Packet: ");
        sprintf(str, "%d", packet->sample_cnt);
        printf(" Sample Counter = %s", str);
        sprintf(str, "%f", packet->yaw);
        printf(" yaw = %s", str);
        sprintf(str, "%f", packet->pitch);
        printf(" pitch = %s", str);
        sprintf(str, "%f", packet->roll);
        printf(" roll = %s\n", str);
        break;
    }
    case GKV_INCLINOMETER_PACKET:
    {
        GKV_InclinometerData* packet;
        packet = (GKV_InclinometerData*)&buf->data;
        printf("Inclinometer Data Packet: ");
        sprintf(str, "%d", packet->sample_cnt);
        printf(" Sample Counter = %s", str);
        sprintf(str, "%f", packet->alfa);
        printf(" alfa = %s", str);
        sprintf(str, "%f", packet->beta);
        printf(" beta = %s\n", str);
        break;
    }
    case GKV_BINS_PACKET:
    {
        GKV_BINSData* packet;
        packet = (GKV_BINSData*)&buf->data;
        printf("BINS Data Packet: ");
        sprintf(str, "%d", packet->sample_cnt);
        printf(" Sample Counter = %s", str);
        sprintf(str, "%f", packet->x);
        printf(" x = %s", str);
        sprintf(str, "%f", packet->y);
        printf(" y = %s", str);
        sprintf(str, "%f", packet->z);
        printf(" z = %s", str);
        sprintf(str, "%f", packet->alfa);
        printf(" alfa = %s", str);
        sprintf(str, "%f", packet->beta);
        printf("beta = %s", str);
        sprintf(str, "%f", packet->q[0]);
        printf(" q0 = %s", str);
        sprintf(str, "%f", packet->q[1]);
        printf(" q1 = %s", str);
        sprintf(str, "%f", packet->q[2]);
        printf(" q2 = %s", str);
        sprintf(str, "%f", packet->q[3]);
        printf(" q3 = %s", str);
        sprintf(str, "%f", packet->yaw);
        printf(" yaw = %s", str);
        sprintf(str, "%f", packet->pitch);
        printf(" pitch = %s", str);
        sprintf(str, "%f", packet->roll);
        printf(" roll = %s\n", str);
        break;
    }
    case GKV_GNSS_PACKET:
    {
        GKV_GpsData* packet;
        packet = (GKV_GpsData*)&buf->data;
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
    case GKV_CUSTOM_PACKET:
    {
        GKV_CustomData* packet;
        packet = (GKV_CustomData*)&buf->data;
        printf("CustomPacket: ");
        for (uint8_t i = 0; i < ((buf->length) / 4); i++)
        {
            if (packet->parameter[i] == packet->parameter[i])// проверка на isnan
            {
                sprintf(str, "%f", (packet->parameter[i]));
                printf(" param = %s", str);
            }
            else
            {
                printf("param = NaN ");
            }
        }
        printf("\n");
        break;
    }
    // Примечание: в данном примере вывод значений некоторых параметров наборного пакета с пометкой int будет некорректен, поскольку данная программа  
    // не посылает запроса на получение номеров парамеров наборного пакета и выводит все параметры, как float.
    }
}
#else
int main()
{
}
#endif