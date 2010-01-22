#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"

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

	while(1) {
		receive();
		send();
	}
}
