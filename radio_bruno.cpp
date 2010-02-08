#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdlib.h>

#define BAUDTTY B38400
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1         //POSIX compliant source

#define NUMINSTR 13
//padroes
#define PDEVICE "/dev/ttyUSB0"
#define PBAUD 9600
#define PDATAB 8
#define PSTOPB 1
#define PPARITY 0
#define PFORMAT 1

#define ESC 0x1B
#define TAB 0x09
#define CRC_POL 0x4599
#define NUMMOTORES 4

void signal_handler_IO (int status);    //definition of signal handler
int wait_flag=1;                     //TRUE while no signal received
int changed=1;

int meu_pow(int a, int b)
{
	int x;
	if (!b) return (1);
	for (x=1;x<b;x++) a*=a;
	return(a);
}

void coloca_na_string(char aux[7+NUMMOTORES],int nrobo, int fm[NUMMOTORES], int drible, int kick)
{

	unsigned int i,x;
	unsigned char aux2;
	unsigned long long int aux1;

	i=0;
	aux[i++]=ESC;
	aux[i++]=nrobo;
	for (x=0;x<NUMMOTORES;i++,x++) aux[i]=abs(fm[x]);
	//i++;
	aux[i]=0;

	switch (drible) {
		case 0: aux[i]&=0b00111111; break;
		case 1: aux[i]|=0b10000000; break;
		case 2: aux[i]|=0b11000000; break;
		default: printf("erro!\n");
	}
	for (x=0;x<NUMMOTORES;x++)
		if (fm[x]<0) {
		 aux[i]|=meu_pow(2,x)&0xFF;
		}
	i++;
	aux2=0;
	for (x=1;x<3+NUMMOTORES;x++) {
		aux2+=aux[x]&0xFF;
	}
	aux[i++]=aux2;

	//agora o CRC
	 //vamos usar long long int pq sizeof(long long int)=8 e temos pelo menos 7 bytes a calcular (3 motores)
	 //vamos torcer pra nao inventarem de colocar mais do que 4 motores no protocolo!
	aux1=0;
	for (x=1;x<4+NUMMOTORES;x++)
	{
	 aux1 <<= 8; //primeiro desloca
	 aux1 |= aux[x]&0xFF; //agora coloca o dado no aux
	}

	aux1 %= CRC_POL; //pegamos o resto da divisao

	aux[i+1] = 0xFF & aux1; //parte low na pos 9
	aux1 >>= 8;
	aux[i] = 0xFF & aux1; //parte high na pos 8

	aux2=0xFFFFFFFF & aux1;

	i+=2;
	aux[i]=TAB;

	return;

}


int main(int argc, char **argv) {

   char *instr[NUMINSTR];

   long BAUD,DATAB,STOPB,PARITY,PARITYON;
   int datab=PDATAB,stopb=PSTOPB,parity=PPARITY,format=PFORMAT,nrobo,drible,kick,baud=PBAUD,fm[NUMMOTORES];
   char *message,device[80]=PDEVICE;

   int fd, res, i, x;
   unsigned int in1;
   struct termios oldtio, newtio;       //place for old and new port settings for serial port
   struct sigaction saio;               //definition of signal action
   char buf[255];                       //buffer for where data is put



   for (x=0;x<NUMINSTR;x++) instr[x]=(char *)malloc(sizeof(char)*300);
   instr[0] ="\r\nOn the command you must include these items in any order, they are:\n";
   instr[1] ="   . -d 'The device name' *  (default: ttyUSB0)   Ex: ttyS0 for com1, ttyS1 for com2, etc\n";
   instr[2] ="   . -r 'Baud Rate'       *  (default: 9600)      Ex: 38400 \n";
   instr[3] ="   . -b 'Data Bits'       *  (default: 8)         Ex: 7, 8 \n";
   instr[4] ="   . -s 'Stop Bits'       *  (default: 1)         Ex: 0 or 1\n";
   instr[5] ="   . -p 'Parity'          *  (default: 0)         Ex: 0=none, 1=odd, 2=even\n";
   instr[6] ="   . -f 'Format received' *  (default: 1)         Ex: 1=hex, 2=dec, 3=ascii\n";
   instr[7] ="   . -m 'Message'\n";
   instr[9] ="     .. Message: 'Robot Nr [0-4]' 'Strenght M1 [0-255]' 'Strenght M2' 'Strenght M3' 'Strenght M4' 'Drible off(0)/on positive(1)/on negative(2)' 'Kick off(0)/on(1)'\n";
   instr[10]="     .. Message Example: 2 -127 127 -127 -127 1 0\n";
   instr[11]=" Items marked with an * are optional.\n";
   instr[12]=" Example command line:  radio -d ttyUSB0 -r 9600 -b 8 -s 0 -p 0 -f 1 -m 2 127 -127 127 -127 1 0\n";


   if ((argc<5+NUMMOTORES)||(argc>17+NUMMOTORES)) {  //if there are the right number of parameters on the command line
   	for (i=0;i<NUMINSTR;i++) printf("%s",instr[i]);
   	printf("Number of arguments incorrect!\n");
   	return(1);
   } else {
		for (i=1; i<argc; i++) {  // for all wild search parameters
			switch (argv[i][0]) {
			 	case '-':
			 		switch (argv[i][1]) {
			 			case 'd':
			 				i++;
			 				if (sscanf(argv[i],"/dev/%s",device)) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the device name!\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 'r':
			 				i++;
			 				baud=atoi(argv[i]);
			 				if (!baud) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the baud rate!\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 'b':
			 				i++;
			 				datab=atoi(argv[i]);
			 				if ((datab!=7)||(datab!=8)) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the data bits!\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 's':
			 				i++;
			 				stopb=atoi(argv[i]);
			 				if ((stopb!=0)||(stopb!=1)) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the stop bits!\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 'p':
			 				i++;
			 				parity=atoi(argv[i]);
			 				if ((parity<0)||(parity>2)) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the parity\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 'f':
			 				i++;
			 				format=atoi(argv[i]);
			 				if ((format<1)||(format>3)) {
				 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 				printf("Fix the format!\n\n");
							 	return(1);
							}
							i++;
							break;
			 			case 'm': //sao nove argumentos
			 				i++;
			 				nrobo=atoi(argv[i++]);
			 				for (x=0;x<NUMMOTORES;x++) fm[x]=atoi(argv[i++]);
			 				drible=atoi(argv[i++]);
			 				kick=atoi(argv[i]);
			 				if ((nrobo<0)||(nrobo>4)||(drible<0)||(drible>2)||(kick<0)||(kick>1)) {
			 					for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
			 					printf("Fix the message1!\n\n");
						 		return(1);
						 	}
						 	for (x=0;x<NUMMOTORES;x++)
						 		if ((fm[x]<-255)||(fm[x]>255)) {
						 			for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
				 					printf("Fix the message!\n\n");
							 		return(1);
							 	}
						 	break;
			 			default:
			 				for (x=0;x<NUMINSTR;x++) printf("%s",instr[x]);
			 				printf("Some parameter mistaken!\n");
						 	return(1);
			 		} //switch what parameter
			 		break;
			 	default:
				 	for (i=0;i<NUMINSTR;i++) printf("%s",instr[i]);
				 	//printf("\n%s\n", argv[i]);
				 	printf("Some parameter mistaken1!\n");
				 	return(1);
		 } //switch (-)
		} //end of all wild parameters

      printf("Device=%s, Baud=%d, Data Bits=%d, Stop Bits=%d, Parity=%d, Format=",device,baud,datab,stopb,parity); //output the received setup parameters
      switch (format) {
      	case 1: printf("hex"); break;
      	case 2: printf("dec"); break;
      	case 3: printf("hex(asc)"); break;
      	case 4: printf("dec(asc)"); break;
      	case 5: printf("asc"); break;
      	default: printf("problemas");
      }
      printf("\nMessage: RobotNumber=%d, ",nrobo);
      for (x=0;x<NUMMOTORES;x++) printf("M%i=%d, ",x,fm[x]);
      printf("Drible=%d, Kick=%d\n",drible,kick);
   }  //end of if-else number of parameters correct

      switch (baud) {
         case 38400: BAUD=B38400; break;
         case 19200: BAUD=B19200; break;
         case 9600:
         default:    BAUD=B9600; break;
         case 4800:  BAUD=B4800; break;
         case 2400:  BAUD=B2400; break;
         case 1800:  BAUD=B1800; break;
         case 1200:  BAUD=B1200; break;
         case 600:   BAUD=B600;  break;
         case 300:   BAUD=B300;  break;
         case 200:   BAUD=B200;  break;
         case 150:   BAUD=B150;  break;
         case 134:   BAUD=B134;  break;
         case 110:   BAUD=B110;  break;
         case 75:    BAUD=B75;   break;
         case 50:    BAUD=B50;   break;
      }  //end of switch baud_rate
      switch (datab) {
         case 8:
         default:  DATAB = CS8;  break;
         case 7:   DATAB = CS7;  break;
         case 6:   DATAB = CS6;  break;
         case 5:   DATAB = CS5;  break;
      }  //end of switch data_bits
      switch (stopb) {
         case 1:  STOPB = 0;      break;
         case 2:
         default: STOPB = CSTOPB; break;
      }  //end of switch stop bits
      switch (parity) {
         case 0:
         default:
            PARITYON = 0;
            PARITY = 0;
            break;
         case 1:                        //odd
            PARITYON = PARENB;
            PARITY = PARODD;
            break;
         case 2:                        //even
            PARITYON = PARENB;
            PARITY = 0;
            break;
      }  //end of switch parity

      //open the device(com port) to be non-blocking (read will return immediately)
      if ((fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
         perror(device);
         return(-1);
      }

      //install the serial handler before making the device asynchronous
      saio.sa_handler = signal_handler_IO;
      sigemptyset(&saio.sa_mask);   //saio.sa_mask = 0;
      saio.sa_flags = 0;
      saio.sa_restorer = NULL;
      sigaction(SIGIO,&saio,NULL);

      // allow the process to receive SIGIO
      fcntl(fd, F_SETOWN, getpid());
      // Make the file descriptor asynchronous (the manual page says only
      // O_APPEND and O_NONBLOCK, will work with F_SETFL...)
      fcntl(fd, F_SETFL, FASYNC);

      tcgetattr(fd,&oldtio); // save current port settings
      // set new port settings for canonical input processing
      newtio.c_cflag = BAUD | CRTSCTS | DATAB | STOPB | PARITYON | PARITY | CLOCAL | CREAD;
      newtio.c_iflag = IGNPAR;
      newtio.c_oflag = 0;
      newtio.c_lflag = 0;       //ICANON;
      newtio.c_cc[VMIN]=1;
      newtio.c_cc[VTIME]=0;
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd,TCSANOW,&newtio);

//	  coloca_na_string(message,nrobo,fm,drible,kick);
      // loop while waiting for input and sending stuff. normally we would do something useful here
      while (1)
      {
      	sleep(2);
      	if (changed) coloca_na_string(message,nrobo,fm,drible,kick);
        x=write(fd,message,2+NUMMOTORES+5);
        if (x==7+NUMMOTORES) {          //write the message to the port
         // after receiving SIGIO, wait_flag = FALSE, input is available and can be read
     //only show what was sent if something changed
//		 printf(".");
         if (changed) {
          printf("\n      |..|NR|");
          for (i=0;i<NUMMOTORES;i++) printf("M%d|",i);
          printf(" O|CS|CRCs |..|\nSent: ");
          for (x=0; x<7+NUMMOTORES; x++)  //for all chars in string
          {
           in1 = message[x]&0xFF;
           switch (format) {
           	case 1:
           	default:
           		printf("|%2X",in1);
           		break;
           	case 2:
         		printf("|%2d",in1);
         		break;
         	case 3:
           		printf("|%2i",in1);
           		break;
           } //end switch format
          }  //end of for all chars in string
          printf("|");
          fflush(stdout);
          changed=0;
         } //if something changed
        } else { //didn't write correctly on serial
        	perror("write");
        	printf("\n\nMensagem com erro (em hexa): ");
        	for (x=0;x<7+NUMMOTORES;x++) printf ("%2X|",message[x]&0xFF);
        }
        if (!wait_flag)  //if input is available
        {
            res = read(fd,buf,255);
            if (res>0)
            {
               printf("\nRecebido: ");
               for (i=0; i<res; i++)  //for all chars in string
               {
                  in1 = message[i]&0xFF;
                  switch (format)
                  {
                     case 1:         //hex
                     default:
                        printf("|%2X",in1);
                        break;
                     case 2:         //decimal
                        printf("|%2d",in1);
                        break;
                     case 3:         //ascii
                        printf ("|%2c",in1);
                        break;
                  }  //end of switch format
               }  //end of for all chars in string
               printf("|");
               fflush(stdout);
            }  //end if res>0
            wait_flag = 1;      // wait for new input
        }  //end if wait flag == FALSE

      }  //while (1)

      // restore old port settings
      tcsetattr(fd,TCSANOW,&oldtio);
      close(fd);        //close the com port

}  //end of main

/***************************************************************************
* signal handler. sets wait_flag to FALSE, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/

void signal_handler_IO (int status)
{
//    printf("received SIGIO signal.\n");
   wait_flag = 0;
}
