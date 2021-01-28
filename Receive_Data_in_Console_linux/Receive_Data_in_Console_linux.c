#ifdef __linux
#include <string.h>
#include <stdlib.h>
#include <unistd.h>     
#include <fcntl.h>      
#include <termios.h> 
#include <stdio.h>
#include <GKV_CommunicationLibrary.h>

int SerialPortHandle;

uint8_t InitSerialPort(char* port_name, int32_t baudrate);
char ReadCOM();
void WriteCOM(PacketBase* buf);
void RecognisePacket(PacketBase* buf);
char* cin(int* length);

int main()
{
    int length = 0;
    /* Select serial port */
    printf("Set Serial Port:");
    char* com_port = cin(&length);
    /* Init GKV Receive Data Structure */
    InitInput InputStructure;
    GKV_Init_Input(&InputStructure);
    /* Connect to Selected serial port*/
    printf("#start connecting to %s\n", com_port);
    if (!(InitSerialPort(com_port, B921600))) return 1;
    /* Read Data From GKV in Main Cycle */
    while (1)
    {
        InputStructure.GKV_Byte = ReadCOM();
        GKV_Process(RecognisePacket, &InputStructure);
    }
    return 0;
}

uint8_t InitSerialPort(char* port_name, int32_t baudrate)
{
    SerialPortHandle = open(port_name, O_RDWR | O_NOCTTY);
    if (SerialPortHandle < 0) {
        printf("Error opening port\n");
        return 0;
    }
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof(tty));
    /* Error Handling */
    if (tcgetattr(SerialPortHandle, &tty) != 0) {
        printf("Error connect termios to port\n");
        return 0;
    }
    /* Save old tty parameters */
    tty_old = tty;
    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t)baudrate);
    cfsetispeed(&tty, (speed_t)baudrate);
    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB;            // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;           // no flow control
    tty.c_cc[VMIN] = 1;                  // read doesn't block
    tty.c_cc[VTIME] = 5;                  // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
    /* Make raw */
    cfmakeraw(&tty);
    /* Flush Port, then applies attributes */
    tcflush(SerialPortHandle, TCIFLUSH);
    if (tcsetattr(SerialPortHandle, TCSANOW, &tty) != 0) {
        printf("Error setting port parameters\n");
        return 0;
    }
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
    int iOut = write(SerialPortHandle, buf, buf->length + 8);
    usleep(1000);
}

char ReadCOM()
{
    char sReceivedChar;
    while (true)
    {
        int iOut = read(SerialPortHandle, &sReceivedChar, 1);
        return sReceivedChar;
    }
    return 0;
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
        uint8_t i;
        printf("CustomPacket: ");
        for (i = 0; i < ((buf->length) / 4); i++)
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