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
#define NUM_ROBOTS 			5
#define WRITE_BYTE_NUMBER	5*NUM_ROBOTS
#define SLEEP_TIME 			7250*5

typedef struct
{
	float force_x;
	float force_y;
	float displacement_theta;
	int kick;
	int chip_kick;
	int drible;
	float current_theta;
	int motorForces[4];
	int id;
	int secret_attack;
} Robot;

void sendToTracker();
void sendToRobots(bool toRadio);
void sendToSimulator();


// >>> HERE BE GLOBALS <<<

//RoboPETServer radiototracker(PORT_RADIO_TO_TRACKER, IP_RADIO_TO_TRACKER);
RoboPETServer radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);
RoboPETClient aitoradio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);
RoboPETClient joytoradio(PORT_JOY_TO_RADIO, IP_JOY_TO_RADIO);

bool real_radio = true;

int CLOCK_WISE_VELOCITY = 15,
    COUNTER_CLOCK_WISE_VELOCITY = 16,
    MIN_DIFF = 30,
	MAX_FORCE = 127;

bool DEBUG = true;
int robot_total = NUM_ROBOTS;
int team_id;

Robot robots[NUM_ROBOTS];
RadioUSB radio;

int ajusteFino[5][3] = {{0}};

// >>> END OF GLOBALS

double toRad(float degrees)
{
	return (degrees * 3.1415) / (float) 180;
}

void giraAnda(int robotIndex)
{
	Vector desl(robots[robotIndex].force_x, -1*robots[robotIndex].force_y);
	Vector normal(cos(robots[robotIndex].current_theta*3.1415/180), sin(robots[robotIndex].current_theta*3.1415/180));


	float angle = desl.angleDegrees(normal);

	printf("Trague-o: %f,%f\n", normal.getX(), normal.getY());
	printf("Angle %f\n", angle);

	 if(angle < MIN_DIFF) {
		printf("ahead ahoy!\n");
		robots[robotIndex].motorForces[0] = MAX_FORCE*6/10;
        robots[robotIndex].motorForces[1] = 0;
        robots[robotIndex].motorForces[2] = (MAX_FORCE*6/10) | 128;
	}
	else if(angle > MIN_DIFF && angle <= 180) {
		//horario
		printf("horario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = int(MAX_FORCE*2/10) | 128;
	}
	else if(angle > MIN_DIFF && angle > 180) {
		//antihorario mano
		printf("antihorario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = MAX_FORCE*2/10;
	}
	else {
		//parado ai
		printf("paradin\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = 0;
	}
	
	for(int i = 0; i < 3; i++)
		robots[robotIndex].motorForces[i] += ajusteFino[robotIndex][i];
	
	if(robots[robotIndex].secret_attack)
		for(int k = 0; k < 3; k++)
			robots[robotIndex].motorForces[k] = MAX_FORCE;
}

void receiveFromAI() {
	
	RoboPET_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		printf("----------------------------");
		printf("Received AI-To-Radio!\n");

		robot_total = packet.aitoradio().robots_size();

		//GAMBIARRA PARA DEIXAR ECP FELIZ
		if (robot_total < 0)
			robot_total = NUM_ROBOTS;

		for(int i=0; i<packet.aitoradio().robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].force_x = packet.aitoradio().robots(i).displacement_x();
			robots[i].force_y = packet.aitoradio().robots(i).displacement_y();
			robots[i].displacement_theta = packet.aitoradio().robots(i).displacement_theta();
			robots[i].kick = packet.aitoradio().robots(i).kick();
			robots[i].chip_kick = packet.aitoradio().robots(i).chip_kick();
			robots[i].drible = packet.aitoradio().robots(i).drible();
			robots[i].id = packet.aitoradio().robots(i).id();
			robots[i].current_theta = packet.aitoradio().robots(i).current_theta();

			if(DEBUG)
				printf("RECEIVE Robot %d: <%f,%f,%f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n", robots[i].id, robots[i].force_x, robots[i].force_y, robots[i].current_theta, robots[i].displacement_theta, robots[i].kick, robots[i].drible, robots[i].chip_kick);
		}
	}
	else
		printf("Didn't receive  AI-To-Radio.\n");
}

void receiveFromJoy() {
	
	RoboPET_WrapperPacket packet;
	if(joytoradio.receive(packet) && packet.has_joytoradio()) {
		
		printf("----------------------------");
		printf("Received Joy-To-Radio!\n");

		AIToRadio aitoradio = packet.joytoradio().aitoradio();
		robot_total = aitoradio.robots_size();

		//GAMBIARRA PARA DEIXAR ECP FELIZ
		if (robot_total < 0)
			robot_total = NUM_ROBOTS;

		for(int i=0; i<aitoradio.robots_size() && i<NUM_ROBOTS; i++)
		{
			robots[i].force_x = aitoradio.robots(i).displacement_x();
			robots[i].force_y = aitoradio.robots(i).displacement_y();
			robots[i].displacement_theta = aitoradio.robots(i).displacement_theta();
			robots[i].kick = aitoradio.robots(i).kick();
			robots[i].chip_kick = aitoradio.robots(i).chip_kick();
			robots[i].drible = aitoradio.robots(i).drible();
			robots[i].id = aitoradio.robots(i).id();
			robots[i].current_theta = aitoradio.robots(i).current_theta();
			
			ajusteFino[robots[i].id][0] = packet.joytoradio().force_0();
			ajusteFino[robots[i].id][1] = packet.joytoradio().force_1();
			ajusteFino[robots[i].id][2] = packet.joytoradio().force_2();
			
			robots[i].secret_attack = packet.joytoradio().secret_attack();

			if(DEBUG) {
				printf("RECEIVE Robot %d: <%f,%f,%f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n", robots[i].id, robots[i].force_x, robots[i].force_y, robots[i].current_theta, robots[i].displacement_theta, robots[i].kick, robots[i].drible, robots[i].chip_kick);
				printf("Ajuste Fino: <%i,%i,%i>\n", ajusteFino[robots[i].id][0], ajusteFino[robots[i].id][1], ajusteFino[robots[i].id][2]);
				if(robots[i].secret_attack)
					printf("\nATAQUE DAS CORUJA !!!\n");
			}
		}

	}
	else
		printf("Didn't receive  Joy-To-Radio.\n");
}

void receive()
{
	receiveFromAI();
	receiveFromJoy();
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
	sendToRobots(real_radio);
}


void sendToRobots(bool toRadio)
{
	/*
	0000 0000 (0%) - 1111 1111 (100%) - 1000 0000 (50%)
	byte 1: índice do robo
	byte 2: força motor
	byte 3: força motor
	byte 4: força motor
	byte 5: chute
	*/

	//Initializes the data to be send for the robot with index i
	unsigned char data_send[WRITE_BYTE_NUMBER]; //data to write
	memset(data_send,0,sizeof data_send);

	for(int i=0; i < robot_total; i+=1)
	{
	    giraAnda(i);

		if(DEBUG)
		{
			printf("SENDING Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n",
			robots[i].id, 
			robots[i].force_x,
			robots[i].force_y, robots[i].displacement_theta,
			robots[i].kick, robots[i].drible,
			robots[i].chip_kick);

			printf("%d = ", robots[i].id+1);

			for(int j=0; j<3; j++) {
				printf("motor(%d): %d - ", j, robots[i].motorForces[j]);
			}

			printf("\n");
		}

		data_send[i*NUM_ROBOTS    ] = robots[i].id+11;
		data_send[i*NUM_ROBOTS + 1] = robots[i].motorForces[0];
		data_send[i*NUM_ROBOTS + 2] = robots[i].motorForces[1];
		data_send[i*NUM_ROBOTS + 3] = robots[i].motorForces[2];
		data_send[i*NUM_ROBOTS + 4] = robots[i].kick;

	}

	if(toRadio) {
		radio.usbSendData( data_send, WRITE_BYTE_NUMBER );
		usleep(SLEEP_TIME);
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

void initialize()
{

    radio = RadioUSB(real_radio);

    radio.usbInitializeDevice();

	aitoradio.open(false);
	joytoradio.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();

	//radiototracker.open(); //not being used yet
	radiotosim.open();


}

void parseOptions(int argc, char **argv)
{
	char ch;

	while((ch = getopt(argc, argv, "jdDt:n:i:p:")) != EOF) {
		switch(ch) {

			case 'j': //abre sock do joy
			break;

			case 'D': DEBUG = true; 
			break;

			case 't': team_id = atoi(optarg); 
			break;

			case 'n': robot_total = atoi(optarg); 
			break;

			case 'd': real_radio = false; 
			break;

//			in the future set the ip and port of the sockets if it makes sense
//			case 'i': hostname = optarg; 
//			break;

//			case 'p': port = atoi(optarg); 
//			break;
		}
	}

}

int main(int argc, char **argv)
{

	printf("Radio Running!\n");

	// initializes everything, sets variables blah blah
	// false denotes that this is a mock connection

	parseOptions(argc,argv);

	initialize( );

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
				sendToRobots(real_radio);
	    }
	}

    radio.usbClosingDevice();
}
