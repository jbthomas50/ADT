#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <vector>

// #include "conversion.h"

typedef union {
    float *f;
    int *i;
    long *l;
    double *d;
    bool *b;
    char *c;
} Data_T;

template <class t>
uint8_t* serialize(t *data)
{
    uint8_t *serialData = new uint8_t [sizeof(t)];
    memcpy((void *)serialData, (void*)data, sizeof(t));
    return serialData;
}

template <class t>
t deserialize(uint8_t *serialData)
{
    t data;
    memcpy(&data, (void*)serialData, sizeof(t));
    return data;
}

void printInstructions(void)
{
    printf("\nCommandline arguments:\n\n");
    printf("\tMultiple values can be entered after each option, but only\n");
    printf("\tone option may be entered.\n\n");
    
    printf("\t-<option 1><option 2> <value(s)>\n\n");
    
    printf("\tOPTION 1:\n");
    printf("\t-s<option2> <n1> <n2> ... <n>\tSend n data\n");
    printf("\t-r<option2> <n>\tReceive n values\n\n");

    printf("\tOPTION 2\n");
    printf("\t-<option 1>f <value(s)>\tSend or receive values of type float\n");
    printf("\t-<option 1>i <value(s)>\tSend or receive values of type int\n");
    printf("\t-<option 1>l <value(s)>\tSend or receive values of type long\n");
    printf("\t-<option 1>d <value(s)>\tSend or receive values of type double\n");
    printf("\t-<option 1>b <value(s)>\tSend or receive values of type bool (When sending, enter bool as number only!)\n");
    printf("\t-<option 1>c <value(s)>\tSend or receive values of type char\n\n");

    printf("\tExample of Sending 3 floats:\n");
    printf("\t\t-sf 10.2 42.0 64.7423\n\n");

    printf("\tExample of receiving 2 integers:\n");
    printf("\t\t-ri 3\n\n");
}

template <class t>
void sendData(t data)
{
    uint8_t *data2send = serialize(&data);

    // TODO: Create code to send data over UART

    printf("Serialized %f to {", data);
    for(uint8_t i = 0; i < sizeof(t); i++)
        printf(" [%x]", data2send[i]);
    printf(" }\n");
    
    delete [] data2send;
}

template <class t>
t receiveData()
{
    // TODO: Create code to receive data over UART
    uint8_t *receivedData;

    // The following couple lines is for testing purposes and should be removed after testing
    t testData = 10.5;
    receivedData = serialize<t>(&testData);

    t data = deserialize<t>(receivedData);

    printf("Deserialized {");
    for(uint8_t i = 0; i < sizeof(t); i++)
        printf(" [%x]", receivedData[i]);
    printf(" } to %f\n", data);

    delete [] receivedData;
    return data;
}

void sendStartFrame()
{
    // TODO: Create code to send start frame over UART
}

void sendEndFrame()
{
    // TODO: Create code to send end frame over UART
}

int main(int argc, char *argv[])
{
    if(argc < 3)
    {
        printInstructions();
        return 0;
    }
    printf("\n");

    uint8_t i = 0;

    if(argv[1][1] == 'r')
    {
        Data_T receivedData;
        int amount = atoi(argv[2]);
        switch(argv[1][2])
        {
            case 'f':
                receivedData.f = new float [argc -2];
                for(; i < amount; i++)
                    receivedData.f[i] = receiveData<float>();
                break;

            case 'i':
                receivedData.i = new int [argc -2];
                for(; i < amount; i++)
                    receivedData.i[i] = receiveData<int>();
                break;

            case 'l':
                receivedData.l = new long [argc -2];
                for(; i < amount; i++)
                    receivedData.l[i] = receiveData<long>();
                break;

            case 'd':
                receivedData.d = new double [argc -2];
                for(; i < amount; i++)
                    receivedData.d[i] = receiveData<double>();
                break;

            case 'c':
                receivedData.c = new char [argc -2];
                for(; i < amount; i++)
                    receivedData.c[i] = receiveData<char>();
                break;

            case 'b':
                receivedData.b = new bool [argc -2];
                for(; i < amount; i++)
                    receivedData.b[i] = receiveData<bool>();
                break;
                break;

            default:
                printf("Option not recognized!\n");
                printInstructions();
                break;
        }
    }
    else /*default to send*/
    {
        // Determine if -s<option 2> or -<option2> was sent
        char dataType;
        if(argv[1][1] == 's')
            dataType = argv[1][2];
        else
            dataType = argv[1][1];
        
        // Send data
        i = 2;
        switch(dataType)
        {
            case 'f':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<float>(atof(argv[i]));

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            case 'i':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<int>(atoi(argv[i]));

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            case 'l':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<long>(atol(argv[i]));

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            case 'd':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<double>(atof(argv[i]));

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            case 'c':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<char>(argv[i][0]);

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            case 'b':
                sendStartFrame();
                for(; i < argc; i++)
                    sendData<bool>(atoi(argv[i]));

                sendEndFrame();
                printf("DATA SENT\n");
                break;

            default:
                printf("Option not recognized!\n");
                printInstructions();
                break;
        }
    }

    printf("\n");
    return 0;
}