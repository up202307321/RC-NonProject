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

void alarmHandler(int signal);
int alarmEnabled = FALSE;
int alarmCount = 0;
enum State {
    Start,
    FLAG,
    A,
    C,
    XOR,
    Final
};
LinkLayer conectionParameters; //global
int Ns = 0;//variavel global  (alternando entre 0 e 1)

int llopen(LinkLayer connectionParameters){
    int fd = openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate);//abrir serial port
    if (fd<0){
        perror("openSerialPort");
        return -1;
    }

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        exit(1);
    }

    /////////////////
    //transmissor
    /////////////////
    if(connectionParameters.role == LlTx){ //se é transmissor
        unsigned char setframe[5] = {0};
        setframe[0] = 0x7E;
        setframe[1] = 0x03;
        setframe[2] = 0x07;
        setframe[3] = setframe[1] ^ setframe[2];
        setframe[4] = 0x7E;

        int tentativas = 0;
        int UArecebido = 0;
        
        while(tentativas < connectionParameters.nRetransmissions && UArecebido == 0){
            
            writeBytesSerialPort(setframe , 5); //enviar set 
            alarm(connectionParameters.timeout);
            alarmEnabled = TRUE;
            unsigned char byte;
            int state = 0;
            enum State currentstate= Start;
            while(alarmEnabled==TRUE && UArecebido==0){ //esperar por UA

                int bytes = readByteSerialPort(&byte); //ler byte
                
                if(bytes > 0){//indica que o byte foi lido

                    switch(currentstate){
                        case Start:
                            if (byte == 0x7E){//flag
                                currentstate = FLAG;
                                break;
                            }
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                                break;
                            }
                        case A:
                            if (byte == 0x07) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else currentstate = Start;
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else currentstate = Start;
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                UArecebido=1;
                                alarm(0); //cancela o alarm
                            }
                            else currentstate = Start;       
                    }
                }
            }
        if (UArecebido==0){
                printf("nao recebi UA");
                tentativas++;
            }
    }
    
    if(UArecebido==1){
        printf("UA recebido");
        return fd;
    }
    else{
        pring("UA nao recebido apos n tentativas");
        closeSerialPort();
        return -1;
    }
    }
    
    /////////////////
    //reciver
    /////////////////
    else {
        unsigned char byte;
        int SETrecebido = 0;
        enum State currentstate= Start;

        while(SETrecebido == 0){ //aguarda o frame SET
            int bytes = readByteSerialPort(&byte);
            if(bytes > 0){
                switch(currentstate){
                        case Start:
                            if (byte == 0x7E){//flag
                                currentstate = FLAG;
                                break;
                            }
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                                break;
                            }
                        case A:
                            if (byte == 0x07) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else currentstate = Start;
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else currentstate = Start;
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                SETrecebido=1;
                  
                            }
                            else currentstate = Start;       

            }
        }
        }
    
        unsigned char UAframe[5] = {0};
        UAframe[0] = 0x7E;
        UAframe[1] = 0x03;
        UAframe[2] = 0x07;
        UAframe[3] = UAframe[1] ^ UAframe[2];
        UAframe[4] = 0x7E;
        writeBytesSerialPort(UAframe, 5);
        return fd;
    }
    return fd;

    }

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    unsigned char bcc2 = 0x00;
    
    ///////////criar frame de informação////////////////
    unsigned char frame[bufSize + 6];
    frame[0] = 0x7E;//F
    frame[1] = 0x03;//A
    frame[2] = (Ns == 0) ? 0x00 : 0x40;//C  0x00 é  I frame com Ns = 0 e 0x40 é I frame com Ns = 1
    frame[3] = frame[1] ^ buf[2]; //BCCI

    for (int i = 0; i< bufSize; i++){
        frame[3+i]= buf[i];
        bcc2 ^= buf[i]; //D1 XOR D2 XOR D3 … XOR DN
    
    }
    frame[bufSize+4]= bcc2;
    frame[bufSize +5]= 0x7E;
    ///////////////////////////////////////////////////
    

    int tentativas = 0;
    int ack= 0;
    int RR=0; //resposta positiva do recetor
    int REJ=0; //resposta negativa do recetor
    while(tentativas < conectionParameters.nRetransmissions && RR==0 ){
        writeBytesSerialPort(&frame , bufSize+6);
        alarm(conectionParameters.timeout);
        unsigned char byte; //armarzena o dado do byte lido
        enum State currentstate = Start;

        int bytes = readByteSerialPort(&byte); //aramazena o sucesso da leitura (0 ou 1)  
        ////////////////////////////////////////////////ver respota do recetor////////////////////////////////////////////////
        if(bytes > 0){//indica que o byte foi lido    
            switch(currentstate){ 
                case Start:
                    if (byte == 0x7E){//flag
                        currentstate = FLAG;
                        break;
                    }
                case FLAG:
                    if (byte == 0x03)//A
                    currentstate = A;
                    else if (byte !=0x7E){
                        currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                        break;
                    }
                case A:
                    if (byte == (Ns == 0 ? 0x05 : 0x85)){ //C//“O valor recebido (byte) é igual ao valor que eu esperava para um RR válido?”
                        RR = 1;  //proxima trama espera  -> byte == 0x85 → RR(1)
                        currentstate = C ;
                    } 
                    else if (byte == (Ns == 0 ? 0x01 : 0x81)) //verifca se é uma rejeicao 
                        currentstate = C;  //0x01 → REJ(0),  0x81 → REJ(1)

                    else if (byte == 0x7E) currentstate = FLAG;

                    else currentstate = Start;
                        break;

                case C:
                    if (byte == 0x03 ^ (Ns == 0 ? 0x05 : 0x85 )) currentstate = XOR; //volta para o inicio
                    else currentstate = Start;
                        break;
                case XOR:
                    if (byte == 0x7E) { //e o final
                        currentstate = Final;
                        break;
                    }
                    else currentstate = Start;       
                }



                if(RR||REJ){
                    ack = 1;
                }
             }
        }
        if(tentativas>0){tentativas++;}

        else if (rek)


    }

    if (ack ==1){
        Ns = 1;
        return 
    }
    else{return -1;}



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

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}
