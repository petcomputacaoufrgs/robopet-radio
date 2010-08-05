#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "rp_client.h"
#include "rp_server.h"
#include "radio.h"
#include "vector.h"

#define PI acos(-1)

RoboPETServer radiototracker(PORT_RADIO_TO_TRACKER, IP_RADIO_TO_TRACKER);
RoboPETClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);

int DEBUG = 1;
int robot_total, robot_remote_control;

typedef struct
{
	float force_x;
	float force_y;
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
	return (degrees * 3.1415) / (float) 180;
}

bool inerciaBreaker = false;
int inerciaCount = 0;
int motor_index_mask[] = {2, 0, 1};


int CLOCK_WISE_VELOCITY = 15,
    COUNTER_CLOCK_WISE_VELOCITY = 16,
    MIN_DIFF = 30;

void giraAnda(int robotIndex)
{
//    int i =robotIndex;
//				printf("Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].displacement_x,
//																	robots[i].displacement_y, robots[i].displacement_theta,
//																	robots[i].kick, robots[i].drible);
    //float MIN_DIFF = 30; //now global

    //printf("(%f, %f)\n", Vector(1.2, 0.2).getX(), Vector(1.4, 4.0).getY());

    Vector desl(robots[robotIndex].force_x, robots[robotIndex].force_y);

    //printf("desl(%f, %f)\n", desl.getX(), desl.getY());

    printf("angleClockwise: %lf\n", desl.angleClockwise());
    if(desl.angleClockwise() < MIN_DIFF || desl.angleClockwise() > 360 - MIN_DIFF)
    {
        printf("ahead!\n");
        robots[robotIndex].motorForces[0] = 0;
        robots[robotIndex].motorForces[1] = 30;
        robots[robotIndex].motorForces[2] = -30;
    }
    else if(desl.angleClockwise() > MIN_DIFF && desl.angleClockwise() < 180)
    {
        printf("turning clockwise...\n");
        for(int i=0; i<3; i++)
            robots[robotIndex].motorForces[i] = -CLOCK_WISE_VELOCITY;
    }
    else
    {
        printf("turning counterclockwise...\n");
        for(int i=0; i<3; i++)
            robots[robotIndex].motorForces[i] = COUNTER_CLOCK_WISE_VELOCITY;
    }

    robots[robotIndex].motorForces[3] = 0;
}

void motionConversion(int robotIndex)
{
	static double motionMatrix[3][3] = {{-sin(toRad(60)), -sin(toRad(30)), -1},
										{0				,	1			 , -1},
										{sin(toRad(60))	, -sin(toRad(30)), -1}};
	double robotInfo[3], acum[3];

    //printf("toRad(60): %lf", toRad(60));
    //printf("sin(toRad(60)): %lf", sin(toRad(60)));

	robotInfo[0] = robots[robotIndex].force_x;
	robotInfo[1] = robots[robotIndex].force_y;
	robotInfo[2] = robots[robotIndex].displacement_theta;
/*
	for(int i = 0; i < 3; i++) {
	    printf("robotInfo[%i]: %lf\n", i, robotInfo[i]);
	}//*/

	for(int i = 0; i < 3; i++) {
		acum[i] = 0;
		for(int j = 0; j < 3; j++) {
			acum[i] += motionMatrix[i][j] * robotInfo[j];
		}
	}
/*
    for(int i = 0; i < 3; i++) {
        printf("acum[%i]: %lf\n", i, acum[i]);
    }//*/

	double max = -99999;
	for(int i=0; i<3; i++)
		if(fabs(acum[i]) > max)
			max = fabs(acum[i]);

    //printf("max: %lf\n", max);

	for(int i=0; i<3; i++)
		if(max != 0)
			acum[i] = 30 * acum[i] / (float) max;
/*
    for(int i = 0; i < 3; i++) {
        printf("acum[%i]: %lf\n", i, acum[i]);
    }//*/

    for(int i = 0; i < 3; i++)
        robots[robotIndex].motorForces[motor_index_mask[i]] = acum[i];
/*
    for(int i = 0; i < 3; i++) {
        printf("robots[%i].motorForces[%i]: %i\n", robotIndex, i, robots[robotIndex].motorForces[i]);
    }//*/

	robots[robotIndex].motorForces[3] = 0;

	//quebra-inércia
#ifdef QUEBRA_INERCIA
	if(!(rand() % 10) && !inerciaBreaker) {
	    inerciaBreaker = true;
	    inerciaCount = 10;
	}

	if(inerciaBreaker && inerciaCount--) {
	    for(int i = 0; i < 3; i++)
	        robots[robotIndex].motorForces[i] = 30;
	}

	if(!inerciaCount) inerciaBreaker = false;
#endif
}


void receive()
{
	RoboPET_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		//printf("----------------------------");
		//printf("Received AI-To-Radio!\n");

		robot_total = packet.aitoradio().robots_size();
		for(int i=0; i<packet.aitoradio().robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].force_x = packet.aitoradio().robots(i).displacement_x();
			robots[i].force_y = packet.aitoradio().robots(i).displacement_y();
			robots[i].displacement_theta = packet.aitoradio().robots(i).displacement_theta();
			robots[i].kick = packet.aitoradio().robots(i).kick();
			robots[i].drible = packet.aitoradio().robots(i).drible();
			robots[i].id = packet.aitoradio().robots(i).id();
			if(DEBUG && false)
				printf("Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].force_x,
																	robots[i].force_y, robots[i].displacement_theta,
																	robots[i].kick, robots[i].drible);
		}
	}
}

void send()
{
	//sendToTracker();
	sendToRobots();
}

void sendToTracker()
{
	RoboPET_WrapperPacket packet;
	RadioToTracker *radiototrackerPacket = packet.mutable_radiototracker();

	radiototrackerPacket->set_nada(0);

	radiototracker.send(packet);
	printf("Sent Radio-To-Tracker\n");
}

void sendToRobots()
{
	for(int i=0; i < robot_total; i++)
	{
	    giraAnda(i);
		//motionConversion(i);
		if(DEBUG)
		{
				printf("Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].force_x,
																	robots[i].force_y, robots[i].displacement_theta,
																	robots[i].kick, robots[i].drible);
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

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
}

void remoteControl()
{
#define TIMES_SEND 100
    printf("robot: %i\n", robot_remote_control);
    robot_total = 1;
    robots[0].id = robot_remote_control - 1;
    robots[0].drible = 0;
    robots[0].kick = 0;
    robots[0].force_x = 0;
    robots[0].force_y = 0;
    robots[0].displacement_theta = 0;

    char c;
    while(1) {
        while(!kbhit())
        {
            send();
        }
        fflush(stdin);
        scanf(" %1c", &c);
        printf("-> %c\n", c);
        Vector v(1, 0);
        int rotation[] = {0, 240, 180, 120, 270, 0, 90, 300, 0, 60};
#define CHAR_TO_DIGIT(x) ((x) - '0')
        if('1' <= c && c <= '9') {
            v.rotate(rotation[CHAR_TO_DIGIT(c)]);
        } else if(c == '0') {
            v = Vector(0, 0);
        }
        robots[0].force_x = v.getX();
        robots[0].force_y = v.getY();
    }
}

int amain(void)
{
  while(!kbhit())
    puts("Press a key!");
  printf("You pressed '%c'!\n", getchar());
  return 0;
}

int main(int argc, char **argv)
{
	printf("Radio Running!\n");

    if(argc > 1) {
        robot_remote_control = atoi(argv[1]);
        printf("robot_remote_control set to %i\n", robot_remote_control);
        if(argc > 4) {
            CLOCK_WISE_VELOCITY = atoi(argv[2]);
            COUNTER_CLOCK_WISE_VELOCITY = atoi(argv[3]);
            MIN_DIFF = atoi(argv[4]);
            printf("CLOCK_WISE_VELOCITY: %i\nCOUNTER_CLOCK_WISE_VELOCITY: %i\nMIN_DIFF = %i\n",
            CLOCK_WISE_VELOCITY, COUNTER_CLOCK_WISE_VELOCITY, MIN_DIFF);
        }
    } else {
        robot_remote_control = 0;
    }

	aitoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();

	radiototracker.open();
	radio.conecta();

    if(robot_remote_control)
        remoteControl();

	clrscr();
	int scrCount = 0;
	while(!kbhit()) {
	    scrCount++;
	    if(scrCount == SCR_CLEAR_DELAY) {
	        scrCount = 0;
	        clrscr();
	    }
		rewindscr();
		receive();
		send();
	}
    while(1) {
        printf("mandando os robos pararem ...\n");
	    for(int i=0; i < robot_total; i++)
	    {
			    for(int j=0; j<3; j++)
				    robots[i].motorForces[j] = 0;

            for(int k = 0; k < 1000; k++)
		        radio.send(robots[i].id+1, robots[i].motorForces, robots[i].drible, robots[i].kick);
	    }
	}
}

