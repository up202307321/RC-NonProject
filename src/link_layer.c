// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

/*typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;
 
int openSerialPort(const char *serialPort, int baudRate)
*/

int llopen(LinkLayer connectionParameters)
{
    //int port = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate )
    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    unsigned char bcc2 = 0x00;
    unsigned char frame[bufSize + 6];
    frame[0] = 0x7E;//F
    frame[1] = 0x03;//A
    frame[2] = 0x00;//C
    frame[3] = frame[1] ^ buf[2]; //BCCI

    for (int i = 0; i< bufSize; i++){
        frame[3+i]= buf[i];
        bcc2 ^= buf[i]; //D1 XOR D2 XOR D3 … XOR DN
    
    }
    frame[bufSize+4]= bcc2;
    frame[bufSize +6]= 0x7E;


    //retransmission


/*
    int tentativas = 0;
    while(tentativas < connectionParameters.nRetransmissions ){

    }
    return 0;
}
*/

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
//Estas funções llwrite() e llread() implementam o Stop & Wait ARQ (Automatic Repeat reQuest) — ou seja, o emissor envia uma trama de dados e só envia a próxima quando o recetor confirma com RR (Receiver Ready).

int llread(unsigned char *packet){

    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose()
{
    // TODO: Implement this function

    return 0;
}


}