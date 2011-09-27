#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <cfloat>

#include "rp_client.h"
#include "rp_server.h"
#include "radio_usb.h"
#include "vector.h"
#include "utils.h"


#define NUM_ROBOTS 			MAX_PLAYERS
#define WRITE_BYTE_NUMBER	6*NUM_ROBOTS
#define SLEEP_TIME 			7250*6

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

void sendToSim();
void sendToTracker();
void sendToRobots(int panic=0);
void sendToSimulator();

void receive();
void receiveFromAI();
void receiveFromJoy();
int kbhit();
int inverteBitCBR2011(int f);



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
int team_id = 0;

Robot robots[NUM_ROBOTS];
//angles bangulo entre frente do robo e eixos dos motores, constantes;
const int theta[] = {60,180,-60,0};
//const int theta[] = {-57,57,135,225};

RadioUSB radio;

int forceAdjustment[5][4] = {{0}};

// >>> END OF GLOBALS

double toRad(float degrees)
{
	return (degrees * RP::PI) / (float) 180;
}

void giraAnda(int robotIndex)
{
	Vector desl(robots[robotIndex].force_x, robots[robotIndex].force_y);
	Vector normal(cos(robots[robotIndex].current_theta*RP::PI/180), sin(robots[robotIndex].current_theta*RP::PI/180));

	float angle = normal.angleCWDegrees(desl);

	printf("Vetor frente: %f,%f\n", normal.getX(), normal.getY());
	printf("Angle %f\n", angle);

	if(robots[robotIndex].displacement_theta != 0) {
		if(robots[robotIndex].displacement_theta > 0) {
			printf("horario\n");
			for(int i=0; i < 3; i++)
				robots[robotIndex].motorForces[i] = -MAX_FORCE*2/10;
		}
		else {
			printf("antihorario\n");
			for(int i=0; i < 3; i++)
				robots[robotIndex].motorForces[i] = MAX_FORCE*2/10;
		}
	}
	else if(desl.getX() == 0 && desl.getY() == 0) {
		//parado ai
		printf("paradin\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = 0;
	}
	else if(angle < MIN_DIFF || angle > 360 - MIN_DIFF) {
		printf("ahead ahoy!\n");
		robots[robotIndex].motorForces[0] = -MAX_FORCE*8/10;
        robots[robotIndex].motorForces[1] = 0;
        robots[robotIndex].motorForces[2] = MAX_FORCE*8/10;
	}
	else if(angle > MIN_DIFF && angle <= 180) {
		//horario
		printf("horario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = -MAX_FORCE*2/10;
	}
	else {
		//antihorario mano
		printf("antihorario\n");
		for(int i=0; i < 3; i++)
			robots[robotIndex].motorForces[i] = MAX_FORCE*2/10;
	}
}

void send()
{
	//sendToTracker();
	sendToSim();
	if(real_radio) sendToRobots();
}

void applyAdjustments(int robotIndex) {

	for(int i = 0; i < 4; i++)
		if(robots[robotIndex].motorForces[i] != 0) {
			if(robots[robotIndex].motorForces[i] > 0)
				robots[robotIndex].motorForces[i] += forceAdjustment[robotIndex][i];
			else
				robots[robotIndex].motorForces[i] -= forceAdjustment[robotIndex][i];
			}
}


int realForce(int force) {

	int trueForce = force;

	if(force > 127)
		trueForce = 127;
	else if(force < -127)
		force = -127;

	if(force < 0)
		trueForce = (-force) | 128;

	return trueForce;
}

void calcForces(int robotIndex) {

	Vector desl(robots[robotIndex].force_x, robots[robotIndex].force_y);
	Vector normal(cos(robots[robotIndex].current_theta*RP::PI/180), sin(robots[robotIndex].current_theta*RP::PI/180));

	float phi = normal.angleCWDegrees(desl);

	printf("Phi: %f\n", phi);

		float major = FLT_MIN;
		float cosins[4] = {0,0,0,0};

		//if we just want to rotate the robot, we clear the cosins in order to
		//make the robot rotate -> (cosins[i] / major) == 0!
		if ((desl.getX() == 0) && (desl.getY() == 0)) {
			major = 1;
			for(int i = 0; i < 3; i++) {
				cosins[i] = 0;
			}
		}
		else {
			for(int i = 0; i < 3; i++) {
				cosins[i] = cos((theta[i] + phi + 90)*RP::PI/180);
				if(abs(cosins[i]) > major)
					major = abs(cosins[i]);
			}
		}

		for(int i = 0; i < 3; i++) {
			robots[robotIndex].motorForces[i] =
				((cosins[i] / major) * (MAX_FORCE*40/100) -
				(robots[robotIndex].displacement_theta /4));
		}
		//teste do motor 4
		robots[robotIndex].motorForces[3] = 85;
		
		//calculando para 4 motores
	/*	if ((desl.getX() == 0) && (desl.getY() == 0)) {
			major = 1;
			for(int i = 0; i < 4; i++) {
				cosins[i] = 0;
			}
		}
		else {
			for(int i = 0; i < 4; i++) {
				cosins[i] = cos((theta[i] + phi + 90)*RP::PI/180);
				if(abs(cosins[i]) > major)
					major = abs(cosins[i]);
			}
		}

		for(int i = 0; i < 4; i++) {
			robots[robotIndex].motorForces[i] =
				((cosins[i] / major) * (MAX_FORCE*40/100) -
				(robots[robotIndex].displacement_theta /4));
		}*/
		
		

}

void sendToRobots(int panic)
{
	/*
	0000 0000 (0%) - 1111 1111 (100%) - 1000 0000 (50%)
	byte 1: índice do robo
	byte 2: força motor
	byte 3: força motor
	byte 4: força motor
	byte 5: força motor
	byte 6: chute
	*/

	//Initializes the data to be send for the robot with index i
	unsigned char data_send[WRITE_BYTE_NUMBER]; //data to write
	memset(data_send,0,sizeof data_send);

	for(int i=0; i < robot_total; i+=1)
	{
		if(!panic)
		{
			//giraAnda(i);
			calcForces(i);

			applyAdjustments(i);

			if(robots[i].secret_attack)
				for(int k = 0; k < 4; k++)
					robots[i].motorForces[k] = MAX_FORCE;
		}
		if(DEBUG) {
			printf("SENDING Robot %d: <%f, %f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n",
			robots[i].id,
			robots[i].force_x,
			robots[i].force_y, 
			robots[i].displacement_theta,
			robots[i].kick, 
			robots[i].drible,
			robots[i].chip_kick);

//			for(int j=0; j<3; j++)
//				printf("adjustement(%d): %d - ", j, forceAdjustment[i][j]);
//			printf("\n");

			for(int j=0; j<4; j++) {
				printf("motor(%d): %d(%d) - ", j, realForce(robots[i].motorForces[j]), robots[i].motorForces[j]);
			}

			printf("\n");
		}


		data_send[i*NUM_ROBOTS    ] = robots[i].id+11;
		data_send[i*NUM_ROBOTS + 1] = realForce(robots[i].motorForces[0]);
		data_send[i*NUM_ROBOTS + 2] = realForce(robots[i].motorForces[1]);
		data_send[i*NUM_ROBOTS + 3] = realForce(robots[i].motorForces[2]);
		data_send[i*NUM_ROBOTS + 4] = realForce(robots[i].motorForces[3]);		
		
/*			
		data_send[i*NUM_ROBOTS + 1] = inverteBitCBR2011(realForce(robots[i].motorForces[0]));
		data_send[i*NUM_ROBOTS + 2] = inverteBitCBR2011(realForce(robots[i].motorForces[1]));
		data_send[i*NUM_ROBOTS + 3] = inverteBitCBR2011(realForce(robots[i].motorForces[2]));
		data_send[i*NUM_ROBOTS + 4] = inverteBitCBR2011(realForce(robots[i].motorForces[3]));

	int mandando = 60;
		data_send[i*NUM_ROBOTS + 1] = inverteBitCBR2011(mandando);
		data_send[i*NUM_ROBOTS + 2] = inverteBitCBR2011(mandando);
		data_send[i*NUM_ROBOTS + 3] = inverteBitCBR2011(mandando);
		data_send[i*NUM_ROBOTS + 4] = inverteBitCBR2011(mandando);
*/	
		data_send[i*NUM_ROBOTS + 5] = robots[i].kick;
		
		//printf("bit invertido: %d \n", inverteBitCBR2011(mandando));

	
	
	}

	radio.usbSendData( data_send, WRITE_BYTE_NUMBER );
	//usleep(SLEEP_TIME);

}

int inverteBitCBR2011(int f)
{//gambiarra master, ecp's fizeram uma placa ao contrario, isso corrige
	int v[8], aux = 0 ,out = 0;
	
	aux = f;
	v[0] = aux%2;
	aux /= 2;
	v[1] = aux%2;
	aux /= 2;
	v[2] = aux%2;
	aux /= 2;
	v[3] = aux%2;
	aux /= 2;
	v[4] = aux%2;
	aux /= 2;
	v[5] = aux%2;
	aux /= 2;
	v[6] = aux%2;
	aux /= 2;
	v[7] = aux%2;
	aux /= 2;
	v[8] = aux%2;
	
	for(int i=8; i>0; i--)
		out += v[8-i]*pow(2,i-1);
		
		return out;
	
}
void initialize()
{

    if(real_radio) {
		radio = RadioUSB(real_radio);
		radio.usbInitializeDevice();
	}

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

	while((ch = getopt(argc, argv, "hqt:n:dm:")) != EOF) {
		
		switch(ch) {
			case 'h':
				// Be sure that this print is updated with all options from this 'switch'.
				printf("Command line options:\n");
				printf(" -q\t\t No on-screen messages. (default=%s\n", (!DEBUG)?"true":"false");
				printf(" -t [int]\t Manually set the team id (default=%i)\n",team_id);
				printf(" -n [int]\t Manually set total of robots (default=%i)\n",NUM_ROBOTS);
				printf(" -d\t\t Don't try to open USB Radio connection.\n");
				printf(" -m [int]\t Manually set max force sent to motors(default=%i).\n",MAX_FORCE);
				exit(0);
				
			case 'q':
				DEBUG = false;
				printf("Quiet Mode ON.\n");
				break;

			case 't':
				team_id = atoi(optarg);
				printf("Team ID = %i\n", team_id);
				break;

			case 'n':
				robot_total = atoi(optarg);
				printf("Total of robots = %i\n)", robot_total);
				break;

			case 'd':
				real_radio = false;
				printf("Simulated Radio mode.\n");
				break;
			
			case 'm':
				MAX_FORCE = atoi(optarg);
				printf("Max Motors Force = %i\n", MAX_FORCE);
				break;
		}
	}

}

int main(int argc, char **argv)
{

	printf("Radio Running!\n");

	// initializes everything, sets variables blah blah
	// false denotes that this is a mock connection

	parseOptions(argc,argv);

	initialize();

	while(!kbhit()) {
		if(DEBUG)
			system("clear");
		receive();
		send();
		usleep(10000);
	}
	
	if(real_radio)
		radio.usbClosingDevice();

      //  while(1) {
        printf("Panic Mode Activated! Stopping all robots...\n");
	    for(int i = 0; i < robot_total; i++)
	    {
			for(int j = 0; j < 4; j++)
				robots[i].motorForces[j] = 0;

            if(real_radio)
			//	for(int k = 0; k < 1000; k++)
					sendToRobots(1);
	    }
//	}
	
		if(real_radio)
		radio.usbClosingDevice();
}


void receive()
{
	receiveFromAI();
	receiveFromJoy();
}


void receiveFromAI() {

	RoboPET_WrapperPacket packet;
	if (aitoradio.receive(packet) && packet.has_aitoradio()) {
		if(DEBUG) {
			printf("----------------------------");
			printf("Received AI-To-Radio!\n");
		}

		robot_total = packet.aitoradio().robots_size();

		team_id = packet.aitoradio().team_id();

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

			if(DEBUG) printf("RECEIVED Robot %d: <%f,%f,%f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n", robots[i].id, robots[i].force_x, robots[i].force_y, robots[i].current_theta, robots[i].displacement_theta, robots[i].kick, robots[i].drible, robots[i].chip_kick);
		}
	}
	else
		if(DEBUG) printf("Didn't receive  AI-To-Radio.\n");
}

void receiveFromJoy() {

	RoboPET_WrapperPacket packet;
	if(joytoradio.receive(packet) && packet.has_joytoradio()) {

		if(DEBUG) {
			printf("----------------------------");
			printf("Received Joy-To-Radio!\n");
		}

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

			forceAdjustment[i][0] = packet.joytoradio().force_0();
			forceAdjustment[i][1] = packet.joytoradio().force_1();
			forceAdjustment[i][2] = packet.joytoradio().force_2();

			robots[i].secret_attack = packet.joytoradio().secret_attack();

			if(DEBUG) {
				printf("RECEIVE Robot %d: <%f,%f,%f> (%f degrees) (Kick = %d) (Drible = %d) (Chip Kick = %d)\n", robots[i].id, robots[i].force_x, robots[i].force_y, robots[i].current_theta, robots[i].displacement_theta, robots[i].kick, robots[i].drible, robots[i].chip_kick);
				printf("Ajuste Fino: <%i,%i,%i>\n", forceAdjustment[robots[i].id][0], forceAdjustment[robots[i].id][1], forceAdjustment[robots[i].id][2]);
				if(robots[i].secret_attack)
					printf("\nATAQUE DAS CORUJA !!!\n");
			}
		}

	}
	else
		if(DEBUG)
			printf("Didn't receive  Joy-To-Radio.\n");
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

		if(DEBUG) printf("SENT Robot %d: forceVector<%f, %f> (%f degrees) (Kick = %d) (Drible = %d)\n", robots[i].id, robots[i].force_x,
																							robots[i].force_y, robots[i].displacement_theta,
																							robots[i].kick, robots[i].drible);
	}

	radiotosim.send(packet);
	if(DEBUG) printf("Sent Radio-To-Simulator\n");
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
