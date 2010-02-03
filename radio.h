#ifndef _ROBOPET_RADIO_H_
#define _ROBOPET_RADIO_H_

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

const int NUM_BAUDS = 15;
const int baudKeys[] = {50, 75, 110, 134, 150, 200, 300, 600, 1200,
						1800, 2400, 4800, 9600, 19200, 38400};
const long baudValues[] = {B50, B75, B110, B134, B150, B200, B300, B600, B1200,
							B1800, B2400, B4800, B9600, B19200, B38400};

const int NUM_DATABITS = 4;
const int databitsKeys[] = {5, 6, 7, 8};
const long databitsValues[] = {CS5, CS6, CS7, CS8};

class Radio
{
	public:
		Radio(){};
		~Radio(){};
		void conecta(const char* device="/dev/ttyUSB0", const int baud=9600,
					const int databits=8, const int stopbits=0, const int parity=0);
		void send(const int robotNumber, int* motorForces, const int drible, const int kick);

	private:
		int fd;
		void coloca_na_string(char aux[7+NUMMOTORES],int nrobo, int fm[NUMMOTORES],
														int drible, int kick);
};

#endif
