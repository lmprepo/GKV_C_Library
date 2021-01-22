#include <windows.h>
#include <stdio.h>
#include <GKV_CommunicationLibrary.h>

HANDLE hSerial;

char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);
char* cin(int* length);

int main()
{
    int length = 0;
    /* Select serial port*/
    printf("Set Serial Port:");
    char* com_port = cin(&length);
    /* Init GKV Receive Data Structure */
    InitInput InputStructure;
    GKV_Init_Input(&InputStructure);
    /* Connect to Selected serial port*/
    printf("#start connecting to %s\n", com_port);
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
    /* Read Data From GKV in Main Cycle */
    while (1)
    {
        InputStructure.GKV_Byte = ReadCOM();
        GKV_Process(RecognisePacket, &InputStructure);
    }
    return 0;
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

void RecognisePacket(PacketBase* buf)
{
    char str[30];

    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        ADCData* packet;
        packet = (ADCData*)&buf->data;
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
        RawData* packet;
        packet = (RawData*)&buf->data;
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
        GyrovertData* packet;
        packet = (GyrovertData*)&buf->data;
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
        InclinometerData* packet;
        packet = (InclinometerData*)&buf->data;
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
        BINSData* packet;
        packet = (BINSData*)&buf->data;
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
    case GKV_CUSTOM_PACKET:
    {
        CustomData* packet;
        packet = (CustomData*)&buf->data;
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