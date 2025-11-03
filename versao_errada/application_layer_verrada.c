// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


/*typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;*/


#define CTRL_START 1
#define CTRL_DATA 2
#define CTRL_END 3   

#define PARAM_FILESIZE 0
#define PARAM_FILENAME 1
#define BAUDRATE 38400

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename){


    printf("application %d\n", nTries);
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.baudRate= baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout= timeout;
    connectionParameters.role = (strcmp(role, "tx") == 0) ? LlTx: LlRx; //tx é transmitter e Rx é Reciver



    //////////transmiter///////////////


    if (connectionParameters.role == LlTx){
        int fd = llopen(connectionParameters);  //abrir ligacao;                  
    if(fd<0){
        printf("erro ao abrir link layer");
    }
        FILE *file = fopen(filename, "rb"); //rb (read em binary mode)

        if(fopen(filename, "rb")==0){
            printf("erro abrir ficheiro (llopen)");
        }
        if(file == 0){
            perror("Erro ao abrir ficheiro");
            llclose();
            return;
        }
        
        //tamanho do ficheiro
        fseek(file, 0L, SEEK_END); //move o ponteiro para o final do arquivo
        long int fileSize = ftell(file); //retorna a posicao atual do ponteiro do aquivo em bytes, contando desde o inicio
        fseek(file, 0L, SEEK_SET); //move o ponteiro para o incio do arquivo 

        ////pacote start 
        unsigned int controlpacket_size;
        unsigned char *controlPacketStart = getControlPacket(CTRL_START, filename, fileSize, &controlpacket_size);
        if (llwrite(controlPacketStart, controlpacket_size) == -1) {
                printf("Erro ao enviar START packet \n");
            }
        free(controlPacketStart);//limpa a memoria reservada para controlPacketStart

        ///////////////enviar pacotes DATA/////////////////
        int number_packet = 0 ;
        unsigned char *data = getData(file, fileSize);
        long int tam_atual = fileSize; //tamanho do que falta enviar

        while(tam_atual >0){
            int datasize = 0;
            if(tam_atual >  MAX_PAYLOAD_SIZE){
                datasize = MAX_PAYLOAD_SIZE;    
            }
            if(tam_atual < MAX_PAYLOAD_SIZE){
                datasize = tam_atual;
            }
            int packetSize;
           unsigned char *packet = getDataPacket(number_packet, data, datasize, &packetSize);

            if (llwrite(packet, packetSize) == -1) {
                    printf("Erro ao mandar DATA packet\n");
                    
                }
            free(packet);
            data += datasize;
            tam_atual -= datasize;
            number_packet = (number_packet+1)%256;            
        }

        ////pacote end 
        unsigned char *controlPacketEnd = getControlPacket(CTRL_END, filename, fileSize, &controlpacket_size);

        if (llwrite(controlPacketEnd, controlpacket_size) == -1) {
                printf("Erro ao enviar END packet");
            }
        free(controlPacketEnd);

        fclose(file);
        llclose();
     }


    //////////Reciver///////////////
    if (connectionParameters.role == LlRx){
        unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE);
        int packetSize = -1;
        unsigned long reciver_packet_size = 0;

        while((packetSize = llread(packet)) <= 0);
        unsigned char *reciver_filename =  parseControlPacket(packet, packetSize, &reciver_packet_size);

        FILE *newfile = fopen((char *)reciver_filename, "wb+");

        if(newfile== 0){
            printf("erro ao abrir ficheiro");
        }

        while (1) {
            while((packetSize==llread(packet )) <= 0);
        
            if (packet[0] == CTRL_END)
                break;

            if(packet[0]==CTRL_DATA){
                unsigned char *buffer = (unsigned char *)malloc(packetSize); 
                parseDataPacket(packet, packetSize, buffer);
                int dataSize = packetSize-4;
                fwrite(buffer, sizeof(unsigned char ), dataSize, newfile);

                free(buffer);
            }
            fclose(newfile);
            free(packet);
            free(reciver_filename);
            llclose();

            break;
        }

    }
}

    //funcoes auxiliares 
    //cria start ou end
    unsigned char *getControlPacket(const unsigned int c, const char *filename, long int length, unsigned int *size){
        const int L1 = sizeof(long int); 
        const int L2 = strlen(filename); //numero de bytes 
        *size = 1 + 2 + L1 + 2 + L2; //tamanho total do pacote
        unsigned char *packet = (unsigned char *)malloc(*size);

        unsigned int pos = 0;
        packet[pos++] = c;                 // Campo C (start ou end)
        packet[pos++] = PARAM_FILESIZE;    // T = 0
        packet[pos++] = L1;                // L (file size)
        memcpy(packet + pos, &length, L1);
        pos += L1;

        packet[pos++] = PARAM_FILENAME;    // T = 1
        packet[pos++] = L2;                // L (filename)
        memcpy(packet + pos, filename, L2);
        return packet;
    }

    //cria pacote data
    unsigned char *getDataPacket(unsigned char sequence, unsigned char *data, int dataSize, int *packetSize){
        *packetSize = 1 + 1 + 2 + dataSize;
        unsigned char *packet = (unsigned char *)malloc(*packetSize);

        packet[0] = CTRL_DATA;
        packet[1] = sequence;
        packet[2] = dataSize >> 8;
        packet[3] = dataSize & 0xFF;
        memcpy(packet + 4, data, dataSize);

        return packet;
    }

    unsigned char *getData(FILE *fd, long int fileLength) //le o conteudo de yn ficheiro para a memoria
    {
        unsigned char *content = (unsigned char *)malloc(fileLength);
        fread(content, sizeof(unsigned char), fileLength, fd);
        return content;
    }

    unsigned char *parseControlPacket(unsigned char *packet, int size, unsigned long int *fileSize)
    {
        int i = 1;
        unsigned char *fileName = NULL;

        while (i < size) {
            unsigned char T = packet[i++];
            unsigned char L = packet[i++];
            if (T == PARAM_FILESIZE) {
                memcpy(fileSize, &packet[i], L);
            } else if (T == PARAM_FILENAME) {
                fileName = (unsigned char *)malloc(L + 1);
                memcpy(fileName, &packet[i], L);
                fileName[L] = '\0';
            }
            i += L;
        }
        return fileName;
    }

    void parseDataPacket(const unsigned char *packet, const unsigned int packetSize, unsigned char *buffer){
        memcpy(buffer, packet + 4, packetSize - 4);
    }

