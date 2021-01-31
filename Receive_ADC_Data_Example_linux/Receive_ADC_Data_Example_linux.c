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
void WriteCOM(GKV_PacketBase* buf);
void RecognisePacket(GKV_PacketBase* buf);
char* cin(int* length);

int main()
{
    int length = 0;
    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm = GKV_ADC_CODES_ALGORITHM;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    uint8_t algorithm_selected = 0;
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
    if (!(InitSerialPort(com_port, B921600))) return 1;
    /* Waiting for device connection and selecting algorithm */
    const int MAX_INCORRECT_CNT = 1000;
    int incorrectCnt = 0;
    while (!(algorithm_selected))
    {
        GKV_SetAlgorithm(&GKV, algorithm);
        Packet_is_Correct = 0;
        while (!(Packet_is_Correct))
        {
            Packet_is_Correct = GKV_ReceiveProcess(&GKV, ReadCOM());
            incorrectCnt++;
            if (incorrectCnt > MAX_INCORRECT_CNT)
            {
               printf( "error too many incorrect packets\n");
               return 1;
            }
        }
        if (GKV.InputBuffer.type == algorithm_packet)
        {
            algorithm_selected = 1;
        }
    }
    printf("#start read loop\n");
    /* Receive Data for Selected algorithm */
    while (1)
    {
        GKV_ReceiveProcess(&GKV, ReadCOM());
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

void WriteCOM(GKV_PacketBase* buf)
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

void RecognisePacket(GKV_PacketBase* buf)
{
    char str[30];

    switch (buf->type)
    {
    case GKV_ADC_CODES_PACKET:
    {
        GKV_ADCData* packet;
        packet = (GKV_ADCData*)&buf->data;
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
    }
}
#else
int main()
{
}
#endif