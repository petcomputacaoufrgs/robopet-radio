#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"
#include "radio.h"

#include <math.h>

#define PI acos(-1)

RoboCupSSLClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);

int DEBUG = 1;

typedef struct
{
	float displacement_x;
	float displacement_y;
	float displacement_theta;
	int kick;
	int drible;
	int motorForces[4];
} Robot;

#define NUM_ROBOTS 5
Robot robots[NUM_ROBOTS];
Radio radio;

double toRad(float degrees)
{
	return (degrees * PI) /180;
}

void motionConversion(int robotIndex)
{
	static double motionMatrix[3][3] = {{-sin(toRad(60)), -sin(toRad(30)), -1},
										{0				,	1			 , -1},
										{sin(toRad(60))	, -sin(toRad(30)), -1}};
	static double robotInfo[3];

	robotInfo[0] = robots[robotIndex].displacement_x;
	robotInfo[1] = robots[robotIndex].displacement_y;
	robotInfo[2] = robots[robotIndex].displacement_theta;

	for(int i = 0; i < 3; i++) {
		robots[robotIndex].motorForces[i] = 0;
		for(int j = 0; j < 3; j++) {
			robots[robotIndex].motorForces[i] += motionMatrix[i][j] * robotInfo[j];
		}
	}
	
	int max = -99999;
	for(int i=0; i<3; i++)
		if(abs(robots[robotIndex].motorForces[i]) > max)
			max = abs(robots[robotIndex].motorForces[i]);
	
	for(int i=0; i<3; i++)
		robots[robotIndex].motorForces[i] = 30 * robots[robotIndex].motorForces[i] / max;

	robots[robotIndex].motorForces[3] = 0;
}


void receive()
{
	SSL_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		printf("----------------------------");
		printf("Received AI-To-Radio!\n");
		for(int i=0; i<packet.aitoradio().robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].displacement_x = packet.aitoradio().robots(i).displacement_x();
			robots[i].displacement_y = packet.aitoradio().robots(i).displacement_y();
			robots[i].displacement_theta = packet.aitoradio().robots(i).displacement_theta();
			robots[i].kick = packet.aitoradio().robots(i).kick();
			robots[i].drible = packet.aitoradio().robots(i).drible();
			if(DEBUG)
				printf("Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].displacement_x, 
																	robots[i].displacement_y, robots[i].displacement_theta, 
																	robots[i].kick, robots[i].drible);
		}
	}
}

void send()
{
	//TODO: send to the real robots
	for(int i=0; i<NUM_ROBOTS; i++)
	{
		motionConversion(i);
		if(DEBUG)
		{
			printf("%d = ", i);
			for(int j=0; j<3; j++)
				printf("motor(%d): %d - ", j, robots[i].motorForces[j]);
			printf("\n");
		}
		//robotNumber, motorForces, drible, kick
		radio.send(i+1, robots[i].motorForces, robots[i].drible, robots[i].kick);
	}
}

int main()
{
	printf("Radio Running!\n");

	aitoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	//TODO: connection with the real robots
	radio.conecta();

	while(1) {
		receive();
		send();
	}
}
