#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"
#include "radio.h"

RoboCupSSLClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);

int DEBUG = 1;

void receive()
{
	SSL_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		printf("----------------------------");
		printf("Received AI-To-Radio!\n");
	}
}

void send()
{
	//TODO: send to the real robots
}

int main()
{
	printf("Radio Running!\n");

	aitoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	//TODO: connection with the real robots
	Radio radio;
	radio.conecta();

	while(1) {
		//robotNumber, motorForces, drible, kick
		char a;
		int forces[4];

		/*
		a = getchar();
		if(a == '1')
		{
			for(int i=0; i<4; i++)
				forces[i] = 30;
		}
		else if(a == '0')
		{
			for(int i=0; i<4; i++)
				forces[i] = 0;
		}
		else if(a == '2')
		{
			forces[0] = 30;
			forces[2] = -30;
			forces[1] = 0;
			forces[3] = 0;
		}
		*/

		radio.send(3, forces, 0, 0);
		receive();
		send();
	}
}
