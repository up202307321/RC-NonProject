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

//Global
LinkLayer conectionParameters;
int alarmEnabled = 0;
int alarmCount = 0;

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
        setframe[2] = 0x03;
        setframe[3] = setframe[1] ^ setframe[2];
        setframe[4] = 0x7E;

        alarmCount = 0;
        int UArecebido = 0;
        
        while(alarmCount < connectionParameters.nRetransmissions && UArecebido == 0){
            
            writeBytesSerialPort(setframe , 5); //enviar set 
            alarm(connectionParameters.timeout);
            alarmEnabled = 1;
            unsigned char byte;
            int state = 0;
            enum State currentstate= Start;
            while(alarmEnabled==1 && UArecebido==0){ //esperar por UA

                int bytes = readByteSerialPort(&byte); //ler byte
                
                if(bytes > 0){//indica que o byte foi lido

                    switch(currentstate){
                        case Start:
                            if (byte == 0x7E){//flag
                                currentstate = FLAG;
                            }
                            break;
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                            }
                            break;
                        case A:
                            if (byte == 0x07) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else {currentstate = Start;}
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else {currentstate = Start;}
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                UArecebido=1;
                                alarm(0); //cancela o alarm
                            }
                            else {currentstate = Start; }      
                    }
                }
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
                            }
                            break;
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                            }
                            break;
                        case A:
                            if (byte == 0x03) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else {currentstate = Start;}
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else {currentstate = Start;}
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                SETrecebido=1;
                  
                            }
                            else {currentstate = Start;}       

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
    return -1;

    }

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize){
    unsigned char bcc2 = 0x00;
    
    ///////////criar frame de informação////////////////
    unsigned char frame[1000];
    frame[0] = 0x7E;//F
    frame[1] = 0x03;//A
    frame[2] = (Ns == 0) ? 0x00 : 0x40;//C  0x00 é  I frame com Ns = 0 e 0x40 é I frame com Ns = 1
    frame[3] = frame[1] ^ frame[2]; //BCCI

    int k = 4;

    for (int i = 0; i< bufSize; i++){
        if(buf[i] == 0x7E) {
            frame[k++] = 0x7D;
            frame[k++] = 0x5E;
        }
        else if(buf[i] == 0x7D) {
            frame[k++] = 0x7D;
            frame[k++] = 0x5D;
        }
        else {frame[k++]= buf[i];}
        bcc2 ^= buf[i]; //D1 XOR D2 XOR D3 … XOR DN
    
    }
    if(bcc2 == 0x7E) {
        frame[k++] = 0x7D;
        frame[k++] = 0x5E;
    }
    else if(bcc2 == 0x7D) {
        frame[k++] = 0x7D;
        frame[k++] = 0x5D;
    }
    else {frame[k++] = bcc2;}
    frame[k++]= 0x7E;
    ///////////////////////////////////////////////////
    

    alarmCount = 0;
    int ack= 0;
    int RR=0; //resposta positiva do recetor
    int REJ=0; //resposta negativa do recetor
    while(alarmCount < conectionParameters.nRetransmissions && RR==0 ){
        writeBytesSerialPort(&frame , k);
        alarm(conectionParameters.timeout);
        unsigned char byte; //armarzena o dado do byte lido
        enum State currentstate = Start;

        while (alarmEnabled && RR==0 && REJ==0){ //enquanto alarm estiver ativado e ainda nao tiver recebido resposta
            int bytes = readByteSerialPort(&byte); //aramazena o sucesso da leitura (0 ou 1)  
            ////////////////////////////////////////////////ver respota do recetor////////////////////////////////////////////////
            if(bytes > 0){//indica que o byte foi lido    
                switch(currentstate){ 
                    case Start:
                        if (byte == 0x7E){//flag
                            currentstate = FLAG;
                        }
                        break;
                    case FLAG:
                        if (byte == 0x03)//A
                        currentstate = A;
                        else if (byte !=0x7E){
                            currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                        }
                        break;
                    case A:
                        if (byte == (Ns == 0 ? 0x05 : 0x85)){ //C//“O valor recebido (byte) é igual ao valor que eu esperava para um RR válido?”
                            RR = 1;  //proxima trama espera  -> byte == 0x85 → RR(1)
                            currentstate = C ;
                        } 
                        else if (byte == (Ns == 0 ? 0x01 : 0x81)) //verifca se é uma rejeicao 
                            currentstate = C;  //0x01 → REJ(0),  0x81 → REJ(1)

                        else if (byte == 0x7E) currentstate = FLAG;

                        else {currentstate = Start;}
                            break;

                    case C:
                        if (byte == 0x03 ^ (Ns == 0 ? 0x05 : 0x85 )) currentstate = XOR; //volta para o inicio
                        else {currentstate = Start;}
                            break;
                    case XOR:
                        if (byte == 0x7E) { //e o final
                            currentstate = Final;
                        }
                        break;
                        else {currentstate = Start;}       
                    }

                    if(RR||REJ){
                        ack = 1;
                        alarm(0);
                    }
                }
            }

        alarm(0);
        if (RR) {
            Ns = 1 - Ns;
            return bufSize;
        }
        else if (REJ) {
            REJ = 0;
        }
        else if (alarmEnabled==0) {
            printf("Timeout");
        }
    }

    return -1;
}



////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
//Estas funções llwrite() e llread() implementam o Stop & Wait ARQ (Automatic Repeat reQuest) — ou seja, o emissor envia uma trama de dados e só envia a próxima quando o recetor confirma com RR (Receiver Ready).

int llread(unsigned char *packet) {
    int Ns_received = 0;
    int dataSize = 0;
    unsigned char bcc2 = 0x00;

    unsigned char byte;
    unsigned char frame[1000];
    int frameIndex = 0;
    enum State currentState = Start;
    int reading = 1;
    
    while (reading) {
        int bytes = readByteSerialPort(&byte);
        if (bytes <= 0) {

            switch (currentState) {
                case Start:
                    if (byte == 0x7E) {
                        currentState = FLAG;
                        frameIndex = 0;
                        frame[frameIndex++] = byte;
                    }
                    break;

                case FLAG:
                    if (byte == 0x03) { // Address field
                        currentState = A;
                        frame[frameIndex++] = byte;
                    } else if (byte != 0x7E) {
                        currentState = Start;
                    }
                    break;

                case A:
                    if (byte == 0x00 || byte == 0x40) { // C field = I frame
                        currentState = C;
                        frame[frameIndex++] = byte;
                        Ns_received = (byte >> 6) & 0x01; // Extract Ns bit
                    } else if (byte == 0x7E) {
                        currentState = FLAG;
                        frameIndex = 1;
                        frame[0] = 0x7E;
                    } else {
                        currentState = Start;
                    }
                    break;

                case C:
                    // Verify BCC1
                    if (byte == (frame[1] ^ frame[2])) {
                        currentState = XOR;
                        frame[frameIndex++] = byte;
                    } else {
                        // BCC1 error → discard
                        currentState = Start;
                        frameIndex = 0;
                    }
                    break;

                case XOR:
                    if (byte == 0x7E) {
                        // Empty data? Probably invalid I frame
                        currentState = FLAG;
                        frameIndex = 1;
                        frame[0] = 0x7E;
                    } else {
                        // Start reading data until flag
                        currentState = Final;
                        frame[frameIndex++] = byte;
                    }
                    break;

                case Final:
                    frame[frameIndex++] = byte;
                    if (byte == 0x7E) {
                        // End of frame reached
                        reading = 0;
                    }
                    break;
            }
        }
    }
    // Frame received

    int i = 4;
    int destuffedIndex = 0;
    unsigned char destuffed[1000];

    // Byte destuffing
    while (i < frameIndex - 1) {
        if (frame[i] == 0x7D) {
            if (frame[i+1] == 0x5E) destuffed[destuffedIndex++] = 0x7E;
            else if (frame[i+1] == 0x5D) destuffed[destuffedIndex++] = 0x7D;
            i += 2;
        } else {
            destuffed[destuffedIndex++] = frame[i++];
        }
    }

    // Last byte is BCC2
    if (destuffedIndex < 1) return -1;
    unsigned char received_bcc2 = destuffed[destuffedIndex - 1];
    destuffedIndex--; // exclude BCC2 from data

    // Compute BCC2
    bcc2 = 0x00;
    for (int j = 0; j < destuffedIndex; j++)
        bcc2 ^= destuffed[j];

    if (bcc2 == received_bcc2) {
        // BCC2 correct
        // Check Ns
        if (Ns_received != Ns) {
            // Duplicate frame: ignore or send RR(Ns)
            unsigned char RR[5] = {0x7E, 0x03, (Ns_received == 0 ? 0x85 : 0x05), 0x00, 0x7E};
            RR[3] = RR[1] ^ RR[2];
            writeBytesSerialPort(RR, 5);
            return 0;
        }

        // New frame OK → send RR(Ns+1)
        unsigned char RR[5] = {0x7E, 0x03, (Ns_received == 0 ? 0x85 : 0x05), 0x00, 0x7E};
        RR[3] = RR[1] ^ RR[2];
        writeBytesSerialPort(RR, 5);

        // Copy data to packet
        memcpy(packet, destuffed, destuffedIndex);
        Ns = 1 - Ns; // toggle Ns
        return destuffedIndex;
    } else {
        // BCC2 error → send REJ(Ns)
        unsigned char REJ[5] = {0x7E, 0x03, (Ns_received == 0 ? 0x81 : 0x01), 0x00, 0x7E};
        REJ[3] = REJ[1] ^ REJ[2];
        writeBytesSerialPort(REJ, 5);
        return -1;
    }
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(){
    /////////////////
    //transmissor
    /////////////////
    if(connectionParameters.role == LlTx){ //se é transmissor
        unsigned char discframe[5] = {0};
        discframe[0] = 0x7E;
        discframe[1] = 0x03;
        discframe[2] = 0x0B;
        discframe[3] = discframe[1] ^ discframe[2];
        discframe[4] = 0x7E;

        alarmCount = 0;
        int UArecebido = 0;
        
        while(alarmCount < connectionParameters.nRetransmissions && UArecebido == 0){
            
            writeBytesSerialPort(discframe , 5); //enviar set 
            alarm(connectionParameters.timeout);
            alarmEnabled = 1;
            unsigned char byte;
            int state = 0;
            enum State currentstate= Start;
            while(alarmEnabled==1 && UArecebido==0){ //esperar por UA

                int bytes = readByteSerialPort(&byte); //ler byte
                
                if(bytes > 0){//indica que o byte foi lido

                    switch(currentstate){
                        case Start:
                            if (byte == 0x7E){//flag
                                currentstate = FLAG;
                            }
                            break;
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                            }
                            break;
                        case A:
                            if (byte == 0x0B) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else {currentstate = Start;}
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else {currentstate = Start;}
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                UArecebido=1;
                                alarm(0); //cancela o alarm
                            }
                            else {currentstate = Start;}       
                    }
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

    int fd = closeSerialPort();//fechar serial port
    if (fd<0){
        perror("closeSerialPort");
        return -1;
    }
    return fd;
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
                            }
                            break;
                        case FLAG:
                            if (byte == 0x03)//A
                                currentstate = A;
                            else if (byte !=0x7E){
                                currentstate = Start; //o voltar para Start, o código descarta bytes inválidos e espera pela próxima flag 0x7E
                            }
                            break;
                        case A:
                            if (byte == 0x0B) currentstate = C ;
                            else if (byte == 0x7E) currentstate = FLAG;
                            else {currentstate = Start;}
                            break;

                        case C:
                            if (byte == 0x7E) currentstate = FLAG; //volta para o inicio
                            else if (byte == (0x03 ^ 0x07)) currentstate = XOR;
                            else {currentstate = Start;}
                            break;
                        case XOR:
                            if (byte == 0x7e) { //e o final
                                currentstate = Final;
                                SETrecebido=1;
                  
                            }
                            else {currentstate = Start;}       

            }
        }
        }
    
        unsigned char discframe[5] = {0};
        discframe[0] = 0x7E;
        discframe[1] = 0x03;
        discframe[2] = 0x0B;
        discframe[3] = discframe[1] ^ discframe[2];
        discframe[4] = 0x7E;
        writeBytesSerialPort(discframe, 5);

        int fd = closeSerialPort();//fechar serial port
        if (fd<0){
            perror("closeSerialPort");
            return -1;
        }
        return fd;
    }
    return -1;
}


void alarmHandler(int signal)
{
    alarmEnabled = 0;
    alarmCount++;

    printf("Alarm #%d received\n", alarmCount);
}
