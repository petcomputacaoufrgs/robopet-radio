#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

#include "rp_client.h"
#include "rp_server.h"
#include "radio_usb.h"
#include "vector.h"

#define PI 					acos(-1)
#define WRITE_BYTE_NUMBER	5 
#define NUM_ROBOTS 			5	

//RoboPETServer radiototracker(PORT_RADIO_TO_TRACKER, IP_RADIO_TO_TRACKER);
RoboPETServer radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);
RoboPETClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);

int DEBUG = 1;
int robot_total = NUM_ROBOTS, robot_remote_control;
int team_id;

typedef struct
{
	float force_x;
	float force_y;
	float displacement_theta;
	int kick;
	int drible;
	float current_theta;
	int motorForces[4];
	int id;
} Robot;

#define NUM_ROBOTS 5
Robot robots[NUM_ROBOTS];
RadioUSB radio;

void sendToTracker();
void sendToRobots();
void sendToSimulator();

double toRad(float degrees)
{
	return (degrees * 3.1415) / (float) 180;
}

bool inerciaBreaker = false;
int inerciaCount = 0;
int motor_index_mask[] = {2, 0, 1};


int CLOCK_WISE_VELOCITY = 15,
    COUNTER_CLOCK_WISE_VELOCITY = 16,
    MIN_DIFF = 40,
	MAX_FORCE = 127;

void giraAnda(int robotIndex)
{
	Vector desl(robots[robotIndex].force_x, -1*robots[robotIndex].force_y);
	Vector normal(cos(robots[robotIndex].current_theta*3.1415/180), sin(robots[robotIndex].current_theta*3.1415/180));

	
	float angle = desl.angle(normal)*180/3.1415;

	printf("Angle %f\n", angle);

	 if(angle < MIN_DIFF) {
		printf("ahead ahoy!\n");
		robots[robotIndex].motorForces[0] = MAX_FORCE*6/10;
        robots[robotIndex].motorForces[1] = 0;
        robots[robotIndex].motorForces[2] = (MAX_FORCE*6/10) | 128;
	}
	else if(angle > MIN_DIFF && angle < 180) {
		//horario
		printf("horario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = int(MAX_FORCE*0.5/10) | 128;
	}
	else {
		//antihorario mano
		printf("antihorario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = MAX_FORCE*2/10;
	}
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

	//for(int i = 0; i < 3; i++) {
	    //printf("robotInfo[%i]: %lf\n", i, robotInfo[i]);
	//}

	for(int i = 0; i < 3; i++) {
		acum[i] = 0;
		for(int j = 0; j < 3; j++) {
			acum[i] += motionMatrix[i][j] * robotInfo[j];
		}
	}

    //for(int i = 0; i < 3; i++) {
        //printf("acum[%i]: %lf\n", i, acum[i]);
    //}

	double max = -99999;
	for(int i=0; i<3; i++)
		if(fabs(acum[i]) > max)
			max = fabs(acum[i]);

    //printf("max: %lf\n", max);

	for(int i=0; i<3; i++)
		if(max != 0)
			acum[i] = 60 * acum[i] / (float) max;

    //for(int i = 0; i < 3; i++) {
    //    printf("acum[%i]: %lf\n", i, acum[i]);
    //}

    for(int i = 0; i < 3; i++) {	
        robots[robotIndex].motorForces[motor_index_mask[i]] = acum[i];
		printf("nao conv: %lf\n", acum[i]);
		if (acum[i] < 0) {
				robots[robotIndex].motorForces[motor_index_mask[i]] = (int) (abs(acum[i])+0.5) | 0x80;
		}
		
		
	}
    //for(int i = 0; i < 3; i++) {
    //    printf("robots[%i].motorForces[%i]: %i\n", robotIndex, i, robots[robotIndex].motorForces[i]);
    //}

	//robots[robotIndex].motorForces[3] = 0;

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
		printf("----------------------------");
		printf("Received AI-To-Radio!\n");

		robot_total = packet.aitoradio().robots_size();
		for(int i=0; i<packet.aitoradio().robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].force_x = packet.aitoradio().robots(i).displacement_x();
			robots[i].force_y = packet.aitoradio().robots(i).displacement_y();
			robots[i].displacement_theta = packet.aitoradio().robots(i).displacement_theta();
			robots[i].kick = packet.aitoradio().robots(i).kick();
			robots[i].drible = packet.aitoradio().robots(i).drible();
			robots[i].id = packet.aitoradio().robots(i).id();
			robots[i].current_theta = packet.aitoradio().robots(i).current_theta();

			if(DEBUG)
				printf("RECEIVE Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].force_x,
																	robots[i].force_y, robots[i].displacement_theta,
																	robots[i].kick, robots[i].drible);
		}
	}
	else
		printf("Didn't receive  AI-To-Radio.\n");
}


/*
void sendToTracker()
{
	RoboPET_WrapperPacket packet;
	RadioToTracker *radiototrackerPacket = packet.mutable_radiototracker();

	radiototrackerPacket->set_nada(0);

	radiototracker.send(packet);
	printf("Sent Radio-To-Tracker\n");
}*/

void sendToSim()
{
	RoboPET_WrapperPacket packet;
	RadioToSim *radiotosimPacket = packet.mutable_radiotosim();
	radiotosimPacket->set_team_id( team_id );

	for(int i=0; i < robot_total; i++) {
		RadioToSim::Robot *r = radiotosimPacket->add_robots();
       	r->set_force_x( robots[i].force_x );
		r->set_force_y( robots[i].force_y );
		r->set_displacement_theta( robots[i].displacement_theta );
		r->set_kick( robots[i].kick );
		r->set_drible( robots[i].drible );
		r->set_id( robots[i].id );
		
		printf("SENT Robot %d: forceVector<%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", robots[i].id, robots[i].force_x,
																							robots[i].force_y, robots[i].displacement_theta,
																							robots[i].kick, robots[i].drible);
	}

	radiotosim.send(packet);
	printf("Sent Radio-To-Simulator\n");
}

void send()
{
	//sendToTracker();
	//sendToSim();
	sendToRobots();
}


void sendToRobots()
{
	/*
	0000 0000 (0%) - 1111 1111 (100%) - 1000 0000 (50%)
	byte 1: índice do robo
	byte 2: força motor
	byte 3: força motor
	byte 4: força motor
	byte 5: chute
	*/	

	for(int i=0; i < robot_total; i++)
	{
	    giraAnda(i);
		//motionConversion(i);
		if(DEBUG)
		{
			printf("SENDING Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", i, robots[i].force_x,
																	robots[i].force_y, robots[i].displacement_theta,
																	robots[i].kick, robots[i].drible);
			printf("%d = ", robots[i].id+1);
			for(int j=0; j<3; j++)
				printf("motor(%d): %d - ", j, robots[i].motorForces[j]);
			printf("\n");
		}
		printf("                                                                     \n \
		                                                                             \n");

		// old Serial Radio protocol
		//robotNumber, motorForces, drible, kick
		//radio.send(robots[i].id+1, robots[i].motorForces, robots[i].drible, robots[i].kick);

		//Initializes the data to be send for the robot with index i
		unsigned char data_send[WRITE_BYTE_NUMBER] = {0}; //data to write
		#define SLEEP_TIME 250000
		//getchar();

		data_send[0] = robots[i].id;
		data_send[1] = robots[i].motorForces[0];
		data_send[2] = robots[i].motorForces[1];
		data_send[3] = robots[i].motorForces[2];
		data_send[4] = robots[i].kick;

		radio.usbSendData( data_send, WRITE_BYTE_NUMBER );
		usleep(SLEEP_TIME);

		
//		radio.usbSendData( data_send, WRITE_BYTE_NUMBER );
//		usleep(SLEEP_TIME);
		//getchar();

//		for(int j = 0; j < sizeof(data_send); j++)
//		{
//			printf("Data[%d]: %d\n",j, data_send[j]);
//		}

		
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

int main(int argc, char **argv)
{
	printf("Radio Running!\n");

	radio.usbInitializeDevice(); //comment me if you want to test the code without the radio plugged in

    if(argc == 2)
		team_id = atoi(argv[1]);
	else {
		team_id = 0;
	}
		//old remote control module. don't delete it, may be useful in near future
        /*else {
			robot_remote_control = atoi(argv[1]);
			printf("robot_remote_control set to %i\n", robot_remote_control);
			if(argc > 4) {
				CLOCK_WISE_VELOCITY = atoi(argv[2]);
				COUNTER_CLOCK_WISE_VELOCITY = atoi(argv[3]);
				MIN_DIFF = atoi(argv[4]);
				printf("CLOCK_WISE_VELOCITY: %i\nCOUNTER_CLOCK_WISE_VELOCITY: %i\nMIN_DIFF = %i\n",
				CLOCK_WISE_VELOCITY, COUNTER_CLOCK_WISE_VELOCITY, MIN_DIFF);
			}
		}
    } else {
        robot_remote_control = 0;
    }*/

	aitoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();

	//radiototracker.open(); //not being used yet
	radiotosim.open();

	/*    
	if(robot_remote_control)
        remoteControl();
	*/

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
	    for(int i = 0; i < robot_total; i++)
	    {
			for(int j = 0; j < 3; j++)
				robots[i].motorForces[j] = 0;

            for(int k = 0; k < 1000; k++)
				sendToRobots();
	    }
	}

	radio.usbClosingDevice();
}

