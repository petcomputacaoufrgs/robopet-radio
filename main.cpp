#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"
#include "radio.h"

#include <math.h>

#define PI acos(-1)

RoboCupSSLServer radiototracker(PORT_RADIO_TO_TRACKER, IP_RADIO_TO_TRACKER);
RoboCupSSLClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);

int DEBUG = 1;
int robot_total;

typedef struct
{
	float displacement_x;
	float displacement_y;
	float displacement_theta;
	int kick;
	int drible;
	int motorForces[4];
	int id;
} Robot;

#define NUM_ROBOTS 5
Robot robots[NUM_ROBOTS];
Radio radio;


void sendToTracker();
void sendToRobots();

double toRad(float degrees)
{
	return (degrees * PI) /180;
}

void motionConversion(int robotIndex)
{
    int motor_index_mask[] = {2, 0, 1};
	static double motionMatrix[3][3] = {{-sin(toRad(60)), -sin(toRad(30)), -1},
										{0				,	1			 , -1},
										{sin(toRad(60))	, -sin(toRad(30)), -1}};
	static double robotInfo[3];

	robotInfo[0] = robots[robotIndex].displacement_x;
	robotInfo[1] = robots[robotIndex].displacement_y;
	robotInfo[2] = robots[robotIndex].displacement_theta;

	for(int i = 0; i < 3; i++) {
		robots[robotIndex].motorForces[motor_index_mask[i]] = 0;
		for(int j = 0; j < 3; j++) {
			robots[robotIndex].motorForces[motor_index_mask[i]] += motionMatrix[i][j] * robotInfo[j];
		}
	}

	int max = -99999;
	for(int i=0; i<3; i++)
		if(abs(robots[robotIndex].motorForces[i]) > max)
			max = abs(robots[robotIndex].motorForces[i]);

	for(int i=0; i<3; i++)
		if(max != 0)
			robots[robotIndex].motorForces[i] = 30 * robots[robotIndex].motorForces[i] / max;

	robots[robotIndex].motorForces[3] = 0;
}


void receive()
{
	SSL_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		//printf("----------------------------");
		//printf("Received AI-To-Radio!\n");

		robot_total = packet.aitoradio().robots_size();
		for(int i=0; i<packet.aitoradio().robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].displacement_x = packet.aitoradio().robots(i).displacement_x();
			robots[i].displacement_y = packet.aitoradio().robots(i).displacement_y();
			robots[i].displacement_theta = packet.aitoradio().robots(i).displacement_theta();
			robots[i].kick = packet.aitoradio().robots(i).kick();
			robots[i].drible = packet.aitoradio().robots(i).drible();
			robots[i].id = packet.aitoradio().robots(i).id();
			if(DEBUG)
				printf("Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].displacement_x,
																	robots[i].displacement_y, robots[i].displacement_theta,
																	robots[i].kick, robots[i].drible);
		}
	}
}

void send()
{
	sendToTracker();
	sendToRobots();
}

void sendToTracker()
{
	SSL_WrapperPacket packet;
	RadioToTracker *radiototrackerPacket = packet.mutable_radiototracker();

	radiototrackerPacket->set_nada(0);

	radiototracker.send(packet);
	printf("Sent Radio-To-Tracker\n");
}

void sendToRobots()
{
	//TODO: send to the real robots
	for(int i=0; i < robot_total; i++)
	{
		motionConversion(i);
		if(DEBUG)
		{
			printf("%d = ", robots[i].id+1);
			for(int j=0; j<3; j++)
				printf("motor(%d): %d - ", j, robots[i].motorForces[j]);
			printf("\n");
		}
		printf("                                                                     \n \
		                                                                             \n");

		//robotNumber, motorForces, drible, kick
		radio.send(robots[i].id+1, robots[i].motorForces, robots[i].drible, robots[i].kick);
	}
}

void remoteControl()
{
    int robot=1;
    printf("Robot Number [1-5]: ");
    scanf("%d", &robot);
    robot_total = 1;
    while(1)
    {
        int x, y;
        char c;
        fflush(stdin);
        scanf(" %c", &c);
        printf("-> %c\n", c);
        switch(c)
        {
            case '8': x = 1;
                      y = 0;
                      break;
            case '4': x = 0;
                      y = -1;
                      break;
            case '6': x = 0;
                      y = 1;
                      break;
            case '2': x = -1;
                      y = 0;
                      break;
            case '5': x = 0;
                      y = 0;
                      break;
        }
        robots[0].displacement_x = x;
        robots[0].displacement_y = y;
        robots[0].displacement_theta = 0;
        robots[0].id = robot-1;
        send();
    }
}

int main()
{
	printf("Radio Running!\n");

	aitoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();

	radiototracker.open();
	//TODO: connection with the real robots
	radio.conecta();

    remoteControl();

	clrscr();
	while(1) {
		rewindscr();
		receive();
		send();
	}
}

