// Link layer protocol implementation

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>


#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FLAG 0x7E
#define ADDRESS_TR 0x03
#define ADDRESS_RT 0x01
#define CONTROL_SET 0x03
#define CONTROL_UA 0x07
#define ESC 0x7D
#define CONTROL_DISC 0x0B
#define C_RR(Nr) ((Nr << 7) | 0x05)
#define C_REJ(Nr) ((Nr << 7) | 0x01)
#define C_N(Ns) (Ns << 6)
#define LL_HDR_SIZE 4

volatile int STOP = FALSE;
int alarmTrig = FALSE;
int alarmCount = 0;
int timeout = 0;
int tries = 0;
static int tramaTx = 0;
static int tramaRx = 0;
void alarmHandler(int signal)
{
    alarmTrig = TRUE;
    alarmCount++;
}

unsigned char getProtocolReply(int fd){
    unsigned char byte, cField = 0;
    stateMachine state = START;
    
    while (alarmTrig == FALSE && state != STOP_R) {  
        if (read(fd, &byte, 1) > 0 || 1) {	// THis is an always on condition???
            switch (state) {
                case START:
                    if (byte == FLAG) state = FLAG_RCV;
                    break;
                case FLAG_RCV:
                    if (byte == ADDRESS_RT) state = ADDRESS_RCV;
                    else if (byte != FLAG) state = START;
                    break;
                case ADDRESS_RCV:
                    if (byte == C_RR(0) || byte == C_RR(1) || byte == C_REJ(0) || byte == C_REJ(1) || byte == CONTROL_DISC){
                        state = CONTROL_RCV;
                        cField = byte;   
                    }
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case CONTROL_RCV:
                    if (byte == (ADDRESS_RT ^ cField)) state = BCC1_OK;
                    else if (byte == FLAG) state = FLAG_RCV;
                    else state = START;
                    break;
                case BCC1_OK:
                    if (byte == FLAG){
                        state = STOP_R;
                    }
                    else state = START;
                    break;
                default: 
                    break;
            }
        } 
    } 
    return cField;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if(fd < 0){
        printf("PANIC! no serial port found\n");
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    //Port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        return -1;
    }

    //memory set for port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0; //Set input mode (non-canonical, no echo,...)
    newtio.c_cc[VTIME] = 0; //Inter-character timer (alarm)
    newtio.c_cc[VMIN] = 0; //Blocking read until 5 chars received

    tcflush(fd, TCIOFLUSH);//cleaning the line

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        return -1;
    }

    //printf("New termios structure set\n");

    stateMachine state = START;
    unsigned char byte;
    timeout = connectionParameters.timeout;
    tries = connectionParameters.nRetransmissions;

    if(connectionParameters.role == LlRx){
         while (state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG){
                                state = FLAG_RCV;
                                printf("LLOPEN: STATE = START->FLAG_RCV\n");
                            } 
                            break;
                        case FLAG_RCV:
                            if (byte == ADDRESS_TR){
                                state = ADDRESS_RCV;
                                printf("LLOPEN: STATE = FLAG_RCV->ADDRESS_RCV\n");
                            }
                            
                            else if (byte != FLAG){
                                state = START;
                                printf("LLOPEN: STATE = FLAG_RCV->START\n");
                            }
                            
                            break;
                        case ADDRESS_RCV:
                            if (byte == CONTROL_SET){
                                state = CONTROL_RCV;
                                printf("LLOPEN: STATE = ADDRESS_RCV->CONTROL_RCV\n");
                            }
                            else if (byte == FLAG){
                                state = FLAG_RCV;
                                printf("LLOPEN: STATE = ADDRESS_RCV->FLAG_RCV\n");
                            }
                            else{
                                state = START;
                                printf("LLOPEN: STATE = ADDRESS_RCV->START\n");
                            } 
                            break;
                        case CONTROL_RCV:
                            if (byte == (ADDRESS_TR ^ CONTROL_SET)){
                                state = BCC1_OK;
                                printf("LLOPEN: STATE = CONTROL_RCV->BCC1_OK\n");
                            }
                            else if (byte == FLAG){
                                state = START;
                                printf("LLOPEN: STATE = CONTROL_RCV->START\n");
                            }
                            break;
                        case BCC1_OK:
                            if (byte == FLAG){
                                state = STOP_R;
                                printf("LLOPEN: STATE = BCC1_OK->STOP_R\n");
                            } 
                            else{
                                state = START;
                                printf("LLOPEN: STATE = BCC1_OK->START\n");
                            }                            
                            break;
                        default: 
                            break;
                    }
                }
            } 
            unsigned char FRAME[5] = {FLAG, ADDRESS_RT, CONTROL_UA, ADDRESS_RT ^ CONTROL_UA, FLAG};
            write(fd, FRAME, 5);
    }
    if(connectionParameters.role == LlTx){

        (void)signal(SIGALRM, alarmHandler);
        
        while (alarmCount < tries && state != STOP_R)
        {
            alarm(timeout);     // Restart ALARM
            alarmTrig = FALSE;
            unsigned char FRAME[5] = {FLAG, ADDRESS_TR, CONTROL_SET, ADDRESS_TR ^ CONTROL_SET, FLAG};
            write(fd, FRAME, 5);
            printf("| TX ----> SET Frame ----> RX |\n");
	
            

            while (alarmTrig == FALSE && state != STOP_R) {
                if (read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG) state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == ADDRESS_RT) state = ADDRESS_RCV;
                            else if (byte != FLAG) state = START;
                            break;
                        case ADDRESS_RCV:
                            if (byte == CONTROL_UA) state = CONTROL_RCV;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case CONTROL_RCV:
                            if (byte == (ADDRESS_RT ^ CONTROL_UA)) state = BCC1_OK;
                            else if (byte == FLAG) state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG) state = STOP_R;
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                }
            }
            if(state == STOP_R){
                printf("| TX <---- UA  Frame <---- RX |\n");
                alarmCount = 0;
            }
            if(alarmTrig == TRUE){
                printf("|      Timeout.  Retry#%d      |\n",alarmCount);
                if(alarmCount > tries){
                    printf("|   Maximum retries reached   |\n");
                    printf("|   Terminating Connection    |\n");
                }
            }
        } 
        //connectionParameters.nRetransmissions--;
    }
    if (state != STOP_R){
                
        return -1;
    } 
      

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(int fd, const unsigned char *buf, int bufSize)
{
    int frameSize = 6 + bufSize;	// Frame size = Application Frame + LinkLayer Header
    unsigned char *frame = (unsigned char *) malloc(frameSize); // Allocate memory for LinkLayer Frame
    
	// Construct Header
	frame[0] = FLAG;
    frame[1] = ADDRESS_TR;
    frame[2] = C_N(tramaTx);
    frame[3] = frame[1] ^frame[2];
	// Add Application Layer Frame as Link Layer Payload 
    memcpy(frame+LL_HDR_SIZE,buf, bufSize);
	// Compute Data Field BCC - BCC2
    unsigned char BCC2 = buf[0];	// Initialize BCC2 field
    for (unsigned int i = 1 ; i < bufSize ; i++)
		BCC2 ^= buf[i];				// BCC2 = BCC2 xor Data[i]

    int j = LL_HDR_SIZE;
	// Add Byte Stuffing when cases required
	// 0x7E [= FLAG]        => {0x7D;0x5E}
	// 0x7D [= Escape Char] => {0x7D;0x5D}
    for (unsigned int i = 0 ; i < bufSize ; i++) {
        if(buf[i] == FLAG || buf[i] == ESC) {
            frame = realloc(frame,++frameSize);	// Add 1 byte to frame size due to use of Byte Stuffing
            frame[j++] = ESC;
            frame[j++] = buf[i]^0x20;	// XOR Data[i] with 0x20
        }
        else
            frame[j++] = buf[i];	// XOR Data[i] with 0x20
    }
    if(BCC2 == FLAG || BCC2 == ESC) {
        frame = realloc(frame,++frameSize);	// Add 1 byte to frame size due to use of Byte Stuffing
        frame[j++] = ESC;
        frame[j++] = BCC2 ^ 0x20;
    }
    else
        frame[j++] = BCC2;				// Add BCC2 field to Frame
    frame[j++] = FLAG;				// Terminate Frame with FLAG char
		
    int iRetry = 0;
    int frameRej = FALSE;
	int frameAck = FALSE;
    alarmTrig   = FALSE;
    while (alarmCount < tries && !frameAck){ 	// External Loop - Control number of retries in case of failled transmission
        alarm(timeout);	
        alarmTrig = FALSE;		// Restart ALARM
        frameRej = 0;			// Boolean to store confirmation of current frame not received by receiver
        frameAck = 0;			// Boolean to store confirmation of current frame been properly received by receiver
        while (alarmTrig == FALSE && !frameAck && !frameRej) {
            write(fd, frame, frameSize);	// Send Information Frame with Data Packet
            switch(buf[0]){
                case 1: 
                    printf("| TX ----> DATA Frame ---> RX |\n");
                    break;
                case 2:
                    printf("| TX ----> STR Frame ----> RX |\n");
                    break;
                case 3:
                    printf("| TX ----> END Frame ----> RX |\n");
                    break;
                default:
                    printf("| TX ----> UNK Frame ----> RX |\n");
                    break;
            }
			unsigned char protocolReply = getProtocolReply(fd);	// ALARM Flag checked inside. Maybe it would no be required here.
            
			// Analysing Protocol Reply
			switch(protocolReply){
				case C_REJ(0):
					frameRej = 1;
					frameAck = 0;
                    printf("| TX <--- REJ0 Frame <---- RX |\n");
					break;
				case C_REJ(1):
					frameRej = 1;
					frameAck = 0;
                    printf("| TX <--- REJ1 Frame <---- RX |\n");
					break;
				case C_RR(0):
					frameRej = 0;
					frameAck = 1;
                    alarmCount = 0;
					tramaTx = (tramaTx+1)%2;
                    printf("| TX <---- RR0 Frame <---- RX |\n");
					break;
				case C_RR(1):
					frameRej = 0;
					frameAck = 1;
                    alarmCount = 0;
					tramaTx = (tramaTx+1) %2;
                    printf("| TX <---- RR1 Frame <---- RX |\n");
					break;	
				default:
					continue;
			}
		}
        iRetry++;
        if(alarmTrig == TRUE){
                printf("|      Timeout.  Retry#%d      |\n",alarmCount);
                if(alarmCount > tries){
                    printf("|   Maximum retries reached   |\n");
                    printf("|   Terminating Connection    |\n");
                }
            }
    }    
    free(frame);
    if(frameAck) 
		return frameSize;
    else{
        return -1;
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(int fd, unsigned char *dataPacket)
{
unsigned char byte, cField;
    int i = 0;
    stateMachine state = START;

    while (state != STOP_R) {  
        if (read(fd, &byte, 1) > 0) {
            printf("LLREAD: Byte[%d]:0x%x \n",i,byte);
            switch (state) {
                case START:
                    if (byte == FLAG) 
						state = FLAG_RCV;
                        printf("LLREAD: STATE = START->FLAG_RCV\n");
                    break;
                case FLAG_RCV:
                    if (byte == ADDRESS_TR){
						state = ADDRESS_RCV;
                        printf("LLREAD: STATE = FLAG:RCV->ADDRESS_RCV\n");
                    }
                    else if (byte != FLAG){
                        state = START;
                        printf("LLREAD: STATE = FLAG_RCV->START\n");
                    }						
                    break;
                case ADDRESS_RCV:
                    if (byte == C_N(0) || byte == C_N(1)){
                        state = CONTROL_RCV;
                        cField = byte;
                        printf("LLREAD: STATE = ADDRESS_RCV->CONTROL_RCV\n");   
                    }
                    else if (byte == FLAG){
                        state = FLAG_RCV;                        
                        printf("LLREAD: STATE = ADDRESS_RCV->FLAG_RCV\n"); 
                    }
                    else if (byte == CONTROL_DISC) {
                        state = CONTROL_RCV;
                        printf("LLREAD: STATE = CONTROL_DISC->CONTROL_RCV\n");
                        cField = byte;
                        unsigned char FRAME[5] = {FLAG, ADDRESS_RT, CONTROL_DISC, ADDRESS_RT ^ CONTROL_DISC, FLAG};
                        write(fd, FRAME, 5);
                        return 0;
                    }
                    else {
                        state = START;
                        printf("LLREAD: STATE = ADDRESS_RCV->START\n");
                    }
                    break;
                case CONTROL_RCV:
                    if ((byte == (ADDRESS_TR^cField))||(byte == (ADDRESS_RT^cField))){ 
						state = READING_DATA;
                        printf("LLREAD: STATE = CONTROL_RCV->READING_DATA\n");
                    }
                    else if (byte == FLAG){ 
						state = FLAG_RCV;                        
                        printf("LLREAD: STATE = CONTROL_RCV->FLAG_RCV\n");
                    }
                    else {
                        state = START;                       
                        printf("LLREAD: STATE = CONTROL_RCV->START\n");
                    }
                    break;
                case READING_DATA:
                    if (byte == ESC){
                        state = DATA_FOUND_ESC;                                               
                        printf("LLREAD: STATE = READING_DATA->DATA_FOUND_ESC\n");
                    } 					
                    else if (byte == FLAG){
                            printf("LLREAD: STATE = READING_DATA:READ FLAG\n");

                            unsigned char bcc2 = dataPacket[i-1];
                            i--;
                            dataPacket[i] = '\0';
                            unsigned char acc = dataPacket[0];

                            for (unsigned int j = 1; j < i; j++)
                                acc ^= dataPacket[j];

                            if (bcc2 == acc){
                                state = STOP_R;
                                unsigned char FRAME[5] = {FLAG, ADDRESS_RT,C_RR(tramaRx), ADDRESS_RT ^ C_RR(tramaRx), FLAG};
                                write(fd, FRAME, 5);

                                tramaRx = (tramaRx + 1)%2;

                                printf("LLREAD: STATE = READING_DATA->STOP_R\n");
                                
                                return i; 
                            }
                    }                        
                    else{
                        dataPacket[i++] = byte;                                          
                        printf("LLREAD: STATE = READING_DATA->READING_DATA\n");
                    }
                    break;
                case DATA_FOUND_ESC:
                    state = READING_DATA;
                    if (byte == 0X5E || byte == 0x5D)
						dataPacket[i++] = byte^0x20;
                    else{
                        printf("LLREAD: STATE = DATA_FOUND_ESC ERROR. Incorrect data received\n");
                        dataPacket[i++] = ESC;
                        dataPacket[i++] = byte;
                    }  
                    printf("LLREAD: STATE = DATA_FOUND_ESC->READING_DATA\n");
                    break;
                default: 
                    break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int fd, int role){
    if(role == LlRx){
        // wait until receive DISC from TX
        stateMachine state = START;
        unsigned char byte;

        while(state != STOP_R){
            if(read(fd, &byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = START->FLAG_RCV\n");
                        }                            
                        break;
                    case FLAG_RCV:
                        if (byte == ADDRESS_TR){
                            state = ADDRESS_RCV;
                            printf("LLCLOSE: STATE = FLAG_RCV->ADDRESS_RCV\n");
                        }                            
                        else if(byte != FLAG){
                            state = START;
                            printf("LLCLOSE: STATE = FLAG_RCV->START\n");  
                        } 
                        break;
                    case ADDRESS_RCV:
                        if (byte == CONTROL_DISC){
                            state = CONTROL_RCV;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->CONTROL_RCV\n");
                        }                             
                        else if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->FLAG_RCV\n");
                        }                                
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->START\n");

                        } 
                        break;
                    case CONTROL_RCV:
                        if (byte == (ADDRESS_RT ^ CONTROL_DISC)){
                            state = BCC1_OK;
                            printf("LLCLOSE: STATE = CONTROL_RCV->BCC1_OK\n");
                        }                            
                        else if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = CONTROL_RCV->FLAG_RCV\n");
                        }                                
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = CONTROL_RCV->START\n");
                        }
                        break;
                    case BCC1_OK:
                        if (byte == FLAG){
                            state = STOP_R;
                            printf("LLCLOSE: STATE = BCC1_OK->STOP_R\n");
                        } 
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = BCC1_OK->START\n");
                        } 
                        break;
                    default: 
                        break;
                }
            }            
        }           
        // Send DISC frame to TX
        unsigned char disconnectRequest[5] = {FLAG, ADDRESS_RT, CONTROL_DISC, ADDRESS_TR ^ CONTROL_DISC, FLAG};
        write(fd, disconnectRequest, 5);
        
        // Wait UA frame from TX to close connection
        state = START;
        byte = 0;

        while(state != STOP_R){
            if(read(fd, &byte, 1) > 0) {
                switch (state) {
                    case START:
                        if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = START->FLAG_RCV\n");
                        }                            
                        break;
                    case FLAG_RCV:
                        if (byte == ADDRESS_TR){
                            state = ADDRESS_RCV;
                            printf("LLCLOSE: STATE = FLAG_RCV->ADDRESS_RCV\n");
                        }                            
                        else if(byte != FLAG){
                            state = START;
                            printf("LLCLOSE: STATE = FLAG_RCV->START\n");  
                        } 
                        break;
                    case ADDRESS_RCV:
                        if (byte == CONTROL_UA){
                            state = CONTROL_RCV;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->CONTROL_RCV\n");
                        }                             
                        else if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->FLAG_RCV\n");
                        }                                
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = ADDRESS_RCV->START\n");

                        } 
                        break;
                    case CONTROL_RCV:
                        if (byte == (ADDRESS_RT ^ CONTROL_DISC)){
                            state = BCC1_OK;
                            printf("LLCLOSE: STATE = CONTROL_RCV->BCC1_OK\n");
                        }                            
                        else if (byte == FLAG){
                            state = FLAG_RCV;
                            printf("LLCLOSE: STATE = CONTROL_RCV->FLAG_RCV\n");
                        }                                
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = CONTROL_RCV->START\n");
                        }
                        break;
                    case BCC1_OK:
                        if (byte == FLAG){
                            state = STOP_R;
                            printf("LLCLOSE: STATE = BCC1_OK->STOP_R\n");
                        } 
                        else{
                            state = START;
                            printf("LLCLOSE: STATE = BCC1_OK->START\n");
                        } 
                        break;
                    default: 
                        break;
                }
            }            
        }
        close(fd);            
        return 1;
    }
    if(role == LlTx){
        //enviar trama de fecho
        unsigned char disconnectRequest[5] = {FLAG, ADDRESS_TR, CONTROL_DISC, ADDRESS_TR ^ CONTROL_DISC, FLAG};
        write(fd, disconnectRequest, 5);
        printf("| TX ----> DISC Frame ---> RX |\n");

        // wait until receive DISC from RX    
        stateMachine state = START;
        unsigned char byte;

        (void)signal(SIGALRM, alarmHandler);
        while (alarmCount <= tries && state != STOP_R){  
            alarm(timeout);
            alarmTrig = FALSE;

            while(alarmTrig == FALSE && state != STOP_R){
                if(read(fd, &byte, 1) > 0) {
                    switch (state) {
                        case START:
                            if (byte == FLAG)
                                state = FLAG_RCV;
                            break;
                        case FLAG_RCV:
                            if (byte == ADDRESS_RT)
                                state = ADDRESS_RCV;
                            else if(byte != FLAG) 
                                    state = START;
                            break;
                        case ADDRESS_RCV:
                            if (byte == CONTROL_UA) 
                                state = CONTROL_RCV;
                            else if (byte == FLAG)
                                    state = FLAG_RCV;
                            else state = START;
                            break;
                        case CONTROL_RCV:
                            if (byte == (ADDRESS_RT ^ CONTROL_UA))
                                state = BCC1_OK;
                            else if (byte == FLAG)
                                    state = FLAG_RCV;
                            else state = START;
                            break;
                        case BCC1_OK:
                            if (byte == FLAG){
                                state = STOP_R;
                                printf("| TX <---- DISC Frame <--- RX |\n");
                            } 
                            else state = START;
                            break;
                        default: 
                            break;
                    }
                }            
            } 
            if (state == STOP_R){
                // Received DISC frame 
                printf("| TX <---- DISC Frame <--- RX |\n");
                // Send UA franme to close connection
                unsigned char disconnectRequest[5] = {FLAG, ADDRESS_TR, CONTROL_UA, ADDRESS_TR ^ CONTROL_UA, FLAG};
                write(fd, disconnectRequest, 5);
                printf("| TX ---->  UA Frame ----> RX |\n");
                close(fd);            
                return 1;
            }   
        }
        if (state != STOP_R) {        
            close(fd);
            return -1;
        }
    }
    return 1;
    
}