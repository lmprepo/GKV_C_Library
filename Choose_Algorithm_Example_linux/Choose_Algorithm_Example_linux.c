#ifdef __linux
#include <string.h>
#include <ctype.h>
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
uint8_t ChooseAlgorithmPacket(uint8_t algorithm);
uint8_t check_input(char* str, int length);
char* cin(int* length);

int main()
{
    int length = 0;

    printf("Set Serial Port:");
    char* com_port = cin(&length);

    printf("#start connecting to %s\n", com_port);

    uint8_t Packet_is_Correct = 0;
    uint8_t algorithm = ADC_CODES_ALGORITHM;
    uint8_t algorithm_packet = GKV_ADC_CODES_PACKET;
    uint8_t algorithm_selected = 0;

    InitInput InputStructure;
    GKV_Init_Input(&InputStructure);
    /* Connect to Selected serial port */
    if (!(InitSerialPort(com_port, B921600))) return 1;

    printf("Choose GKV algorithm:\n");
    printf("0 - ADC Codes Data from Sensors\n");
    printf("1 - Calibrated Raw Sensor Data\n");
    printf("2 - Orientation Kalman Filtered Data\n");
    printf("4 - Inclinometer Data\n");
    printf("5 - Orientation Mahony Filtered Data\n");
    printf("6 - BINS Navigation Data\n");
    printf("7 - Custom Algorithm\n");
    printf("8 - Navigation Data GNSS+BINS\n");
    printf("9 - Navigation Data GNSS+BINS type 2\n");

    printf( "Selected Algorithm = ");
    /* Select Algorithm Number and Check It */
    char* alg_string = NULL;
    do
    {
        alg_string = cin(&length);
        if (check_input(alg_string,length))
        {
            printf( "Selected Algorithm = %s\n", &alg_string);
            algorithm = atoi(alg_string);
        }
        else
        {
            printf("Wrong value. Try Again. Selected Algorithm = ");
            algorithm = 255;
        }
    } while (!(check_input(alg_string, length)));

    algorithm_packet = ChooseAlgorithmPacket(algorithm);
    //Set_Custom_Packet_Params(WriteCOM, sizeof(custom_parameters), &custom_parameters[0]);//функция настройки параметров наборного пакета

    /* Waiting for device connection and selecting algorithm */
    while (!(algorithm_selected))
    {
        Set_Default_Algorithm_Packet(WriteCOM);//функция выбора пакета алгоритма по умолчанию
        //Set_Custom_Algorithm_Packet(WriteCOM);//функция выбора наборного пакета
        Set_Algorithm(WriteCOM, algorithm);
        Packet_is_Correct = 0;
        while (!(Packet_is_Correct))
        {
            InputStructure.GKV_Byte = ReadCOM();
            Packet_is_Correct = GKV_Process(RecognisePacket, &InputStructure);
        }
        if (InputStructure.InputPacket.type == algorithm_packet)
        {
            algorithm_selected = 1;
        }
    }
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

uint8_t check_input(char* str, int length)
{
    if (!(length == 1))
    {
        return 0;
    }
    char char_array[2];
    if (!(isdigit(str[0])))
    {
        return 0;
    }
    if ((atoi(str) > 9) || (atoi(str) == 3))
    {
        return 0;
    }
    return 1;
}

uint8_t ChooseAlgorithmPacket(uint8_t algorithm)
{
    switch (algorithm)
    {
    case ADC_CODES_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case SENSORS_DATA_ALGORITHM:
    {
        return GKV_RAW_DATA_PACKET;
        break;
    }
    case ORIENTATION_KALMAN_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case INCLINOMETER_ALGORITHM:
    {
        return GKV_INCLINOMETER_PACKET;
        break;
    }
    case ORIENTATION_MAHONY_ALGORITHM:
    {
        return GKV_EULER_ANGLES_PACKET;
        break;
    }
    case BINS_NAVIGATON_ALGORITHM:
    {
        return GKV_BINS_PACKET;
        break;
    }
    case CUSTOM_ALGORITHM:
    {
        return GKV_ADC_CODES_PACKET;
        break;
    }
    case KALMAN_GNSS_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    case ESKF5_NAVIGATON_ALGORITHM:
    {
        return GKV_GNSS_PACKET;
        break;
    }
    default:
        return GKV_ADC_CODES_PACKET;
        break;
    }
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
        printf("CustomPacket: ");
        uint8_t i;
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