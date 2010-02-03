#include "radio.h"

int meu_pow(int a, int b)
{
	int x;
	if (b==0)
		return (1);
	for (x=1; x<b; x++)
		a*=a;
	return a;
}

/***************************************************************************
* signal handler. sets wait_flag to FALSE, to indicate above loop that     *
* characters have been received.                                           *
***************************************************************************/
void signal_handler_IO (int status)
{
//    printf("received SIGIO signal.\n");
   int wait_flag = 0;
}

void Radio::coloca_na_string(char aux[7+NUMMOTORES],int nrobo, int fm[NUMMOTORES], int drible, int kick)
{
	unsigned int i,x;
	unsigned char aux2;
	unsigned long long int aux1;

	i=0;
	aux[i++]=ESC;
	aux[i++]=nrobo;
	for (x=0;x<NUMMOTORES;i++,x++)
		aux[i]=abs(fm[x]);
	//i++;
	aux[i]=0;

	switch (drible) {
		case 0: aux[i]&=0b00111111; break;
		case 1: aux[i]|=0b10000000; break;
		case 2: aux[i]|=0b11000000; break;
		default: printf("erro!\n");
	}
	for (x=0;x<NUMMOTORES;x++)
		if (fm[x]<0)
		{
			aux[i]|=meu_pow(2,x)&0xFF;
		}
	i++;
	aux2=0;
	for (x=1;x<3+NUMMOTORES;x++)
		aux2+=aux[x]&0xFF;
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
}

void Radio::conecta(const char* device, const int baud, const int databits,
										const int stopbits, const int parity)
{
	struct termios oldtio, newtio;
	struct sigaction saio;
	long BAUD,DATAB,STOPB,PARITY,PARITYON;

	BAUD = B9600;
	for(int i=0; i<NUM_BAUDS; i++)
		if(baud == baudKeys[i])
		{
			BAUD = baudValues[i];
			break;
		}

	DATAB = CS8;
	for(int i=0; i<NUM_DATABITS; i++)
		if(databits == databitsKeys[i])
		{
			DATAB = databitsValues[i];
			break;
		}

	if(stopbits == 1)
		STOPB = 0;
	else
		STOPB = CSTOPB;

	if(parity == 1) //odd
	{
        PARITYON = PARENB;
        PARITY = PARODD;
	}
	else if(parity == 2) //even
	{
		PARITYON = PARENB;
		PARITY = 0;
	}
	else
	{
		PARITYON = 0;
        PARITY = 0;
	}

	//open the device(com port) to be non-blocking (read will return immediately)
	if ((fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0) {
		perror(device);
		//return -1;
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
}

void Radio::send(const int robotNumber, int* motorForces, const int drible, const int kick)
{
	int x;
	char message[500];
	coloca_na_string(message, robotNumber, motorForces, drible, kick);
    x=write(fd, message, 2+NUMMOTORES+5);
}
