// Application layer protocol implementation
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <math.h>

#include "link_layer.h"
#include "application_layer.h"

//MISC
#define DATA_PKT        1
#define DATA_PKT_START  2
#define DATA_PKT_END    3
#define CTRL_PKT_START  2
#define CTRL_PKT_END    3


void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *fName)
{
    //tenho de montar o struct dos conectionParameters
    printf("in application layer\n");
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort,serialPort);
    connectionParameters.role = strcmp(role, "tx") ? LlRx : LlTx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

	// Open Serial Port
    printf("|=============================|\n");
    printf("|========= llopen() ==========|\n");
    printf("|=============================|\n");
    int fd = llopen(connectionParameters);
    
    if (fd < 0){
        printf("Error on connection\n");
        exit(-1);
    }
	// Role is Tx
    if(connectionParameters.role == LlTx){
        
		// Open file
		FILE* fp = fopen(fName, "rb");
        if (fp == NULL){
            printf("File not found. Terminating\n");
            exit(-1);
		}
		// Determine file size
        fseek(fp, 0, SEEK_END);		// read file till eof
        long fileSize = ftell(fp); 	// store file size
        fseek(fp, 0, SEEK_SET);		// reset file to initial position
        //printf(fName, fileSize);
       		   
		// Prepare START Packet Payoload
		unsigned int pktSize;
		unsigned char *pktPayload = prepAppCtrlPacketPayload(CTRL_PKT_START, fName, fileSize, &pktSize);
		
        printf("|=============================|\n");
        printf("|========= llwrite() =========|\n");
        printf("|=============================|\n");

		// Send START Packet
		if(llwrite(fd, pktPayload, pktSize) == -1){ 
			printf("Exit: Error in START packet\n");
			exit(-1);
		}
		
		unsigned char Ns = 0;
        long fSize = fileSize;
        int dataChunkSize;

        // Reserve memory to store data read from file
        unsigned char* dataBuf = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);

        // While not End of File
		while (fSize>0)
		{
			// Read file data chunks
			dataChunkSize = fread(dataBuf, sizeof(unsigned char),(size_t)MAX_PAYLOAD_SIZE, fp);
            fSize-=dataChunkSize;
			// Prepare DATA Packet
			pktPayload = prepAppDataPacketPayload(Ns, dataBuf, dataChunkSize, &pktSize);
			// Send Data Packet
			if(llwrite(fd, pktPayload, pktSize) == -1){ 
                printf("Exit: Error in Data packet\n");
                exit(-1);
            }
			Ns = (Ns + 1) % 2;
		}	
		// Relese buffer memory allocated
		free(dataBuf);
		
		// Prepare END Packet
		pktPayload = prepAppCtrlPacketPayload(CTRL_PKT_END, fName, fileSize, &pktSize);
		
		// Send END Packet
		if(llwrite(fd, pktPayload, pktSize) == -1){ 
			printf("Exit: Error in END packet\n");
			exit(-1);
		}
		
        printf("|=============================|\n");
        printf("|========= llclose() =========|\n");
        printf("|=============================|\n");
		
        // Close Connection
		llclose(fd,connectionParameters.role);
	}
	// Role is Rx
    else if(connectionParameters.role == LlRx){
		unsigned char *packet = (unsigned char *)malloc(MAX_PAYLOAD_SIZE+3);
		int packetSize = -1;
        printf("|=============================|\n");
        printf("|========= llread() ==========|\n");
        printf("|=============================|\n");   
        unsigned long fSizeRxStr = 0;
        unsigned long fSizeRxEnd = 0;
        int nBytesFileSize;
        int nBytesFileName;
        unsigned char *fNameStr;
        while ((packetSize = llread(fd, packet)) < 0);
		if(packetSize == 0) 
            llclose(fd,connectionParameters.role);

		//unsigned char* name = parseControlPacket(packet, packetSize, &rxFileSize); 

        //Receive Data Start Packet
        if(packet[0] == DATA_PKT_START){   
            //recieved "start frame"
            int iPkt = 1; // Temporary variable to store and decrement a pointer to backet position
            while(iPkt < packetSize){
                //printf("packet size: %d and iPkt %d", packetSize, iPkt);
                if(packet[iPkt] == 0){ // File size parameter
                    nBytesFileSize = packet[++iPkt];
                    for (int idx = 0 ; idx < nBytesFileSize ; idx++){
                        fSizeRxStr = packet[++iPkt]*256^(nBytesFileSize-1-idx);
                    }

                }
                else if (packet[++iPkt] == 1){    // File Name parameter
                    nBytesFileName = packet[++iPkt];
                    // allocate memory for filename storage
                    fNameStr = (void*)malloc(nBytesFileName);
                    // Copy filename to fName
                    memcpy(fNameStr,(const void *)&packet[++iPkt],nBytesFileName);  
                }
            }
            printf("LLREAD:RX File Size: %d File Name: %s\n", fSizeRxStr,fName);
            printf("\n");
        }
        // Create File
		FILE* newFile = fopen((char *) fName, "wb+");
        if(newFile == NULL){
            printf("LLREAD:RX Could not open new file\n");
            exit(-1);
        }
        unsigned long rxDataSize = 0;
		while ((fSizeRxStr-rxDataSize) > 0) {
            while ((packetSize = llread(fd, packet)) < 0);
            // Receive Data Info Packet
            if(packet[0] == DATA_PKT){
                //recieve data frame
                printf("LLREAD:RX packetSize:%d packet[0]: %d\n",packetSize,packet[0]);
                // Allocate memory for APP Frame Payload
				unsigned char *buffer = (void*)malloc(packetSize-3);
                memcpy(buffer,(const void *)&packet[3],packetSize-3);
                // APP Payload size consistency
                if(packetSize == 256*packet[1]+packet[2])
                    printf("INFO: APP HEADER Info consistent with APP DATA Payload size\n");
                else
                    printf("ERROR: APP HEADER Info inconsistent with APP DATA Payload size\n");
                fwrite((const void *) buffer, sizeof(unsigned char), packetSize-3, newFile);
                rxDataSize += (packetSize-3);
                // Relase memory allocated for APP Frame Payload
                free(buffer);
			}
            // Receive Data End Packet
            else if(packet[0] == DATA_PKT_END){
                //recieved "start frame"
                int iPkt = 1; // Temporary variable to store and decrement a pointer to packet position
                while(iPkt < packetSize){
                    if(packet[iPkt] == 0){ // File size parameter
                        if(packet[++iPkt] == nBytesFileSize){
                            for (int idx = 0 ; idx < nBytesFileSize ; idx++){
                                fSizeRxEnd  = packet[++iPkt]*256^(nBytesFileSize-1-idx);
                            }
                            if(fSizeRxEnd == fSizeRxStr){
                                printf("INFO: DATA START and END packets with consistent file size information\n");
                            }
                            else{
                                printf("INFO: DATA START and END packets with inconsistent file size information\n");
                            }                            
                        }
                        else{
                            printf("INFO: DATA START and END packets with inconsistent file size information.\n");
                        }                        
                    }
                    else if (packet[iPkt] == 1){    // File Name parameter
                        if(packet[++iPkt] == nBytesFileSize){
                            // allocate memory for filename storage
                            unsigned char *fNameEnd = (void*)malloc(nBytesFileName);
                            // Compare filename to fName
                            int idx = 0;
                            while( idx < nBytesFileName){
                                if(fNameEnd[idx] != fNameStr[idx++]){
                                    printf("INFO: DATA START and END packets with inconsistent file name information\n");
                                    break;
                                }
                                printf("INFO: DATA START and END packets with consistent file name information\n"); 
                            }
                        }
                        else{
                            printf("INFO: DATA START and END packets with inconsistent file name information\n");
                        }                       
                    }
                }
                printf("LLREAD:RX File Syze: %d File Name: %s\n", fSizeRxStr,fName);
                printf('\n');
                continue;
            }
			else{
                //recieve "Unknown Frame Type"
                printf("INFO: Received Unknown Frame Type\n");
            }                
		}
        // Received file completed. Close file
		fclose(newFile);
        // Close connection
        llclose(fd,connectionParameters.role);
	}
	else{
        printf("Error, undefined role\n");
        exit(-1);
	}
}
        
unsigned char * prepAppCtrlPacketPayload(const unsigned int c, const char* filename, long length, unsigned int* size){

    const unsigned char L1 = 4;
    const unsigned char L2 = strlen(filename);
    *size = 1+2+L1+2+L2;
    unsigned char *packet = (unsigned char*)malloc(*size);
    
    unsigned int pos = 0;
    packet[pos++]=c;
    packet[pos++]=0;
    packet[pos++]=L1;

    for (unsigned char i = 0 ; i < L1 ; i++) {
        packet[3+L1-1-i] = length & 0xFF;
        length >>= 8;
    }
    pos+=L1;
    packet[pos++]=1;
    packet[pos++]=L2;
    memcpy(packet+pos, filename, L2);
    return packet;
}

unsigned char * prepAppDataPacketPayload(unsigned char sequence, unsigned char *data, unsigned int dataSize, int *packetSize){

    *packetSize = 3 + dataSize;
    //printf("LLWRITE:prepAppDataPacketPayload PacketSize: %d",*packetSize);
    unsigned char* packet = (unsigned char*)malloc(*packetSize);

    packet[0] = 1;  
    packet[1] = dataSize >> 8 & 0xFF;
    packet[2] = dataSize & 0xFF;
    memcpy(packet+3, data, dataSize);

    return packet;
}

unsigned char* parseControlPacket(unsigned char* packet, int size, unsigned long int *fileSize) {

    // File Size
    unsigned char fileSizeNBytes = packet[2];
    unsigned char fileSizeAux[fileSizeNBytes];
    memcpy(fileSizeAux, packet+3, fileSizeNBytes);
    for(unsigned int i = 0; i < fileSizeNBytes; i++)
        *fileSize |= (fileSizeAux[fileSizeNBytes-i-1] << (8*i));

    // File Name
    unsigned char fileNameNBytes = packet[3+fileSizeNBytes+1];
    unsigned char *name = (unsigned char*)malloc(fileNameNBytes);
    memcpy(name, packet+3+fileSizeNBytes+2, fileNameNBytes);
    return name;
}
	
void parseDataPacket(const unsigned char* packet, const unsigned int packetSize, unsigned char* buffer) {
    memcpy(buffer,packet+3,packetSize-3);
    buffer += packetSize+3;
}				



