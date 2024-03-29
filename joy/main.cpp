#include <GL/glut.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "joy.h"
#include "rp_server.h"
#include "vector.h"
#include <ctime>

#include <fcntl.h>
#include <linux/joystick.h>

#define INC 1
#define DEC -1

#define MAX_BOT 5
#define MAX_MOTOR 3
#define MAX_BOT_INDEX MAX_BOT - 1
#define MAX_MOTOR_INDEX MAX_MOTOR - 1
#define DEBUG 1

RoboPETServer joyToRadio(PORT_JOY_TO_RADIO, IP_JOY_TO_RADIO);
int current_bot = 0;
int current_motor = 0;
int forces[5][3] = {{0}};

void sendToRadio(Joystick &joy) {

    RoboPET_WrapperPacket packet;

    JoyToRadio *joytoradioPacket = packet.mutable_joytoradio();
	AIToRadio *aitoradioPacket = joytoradioPacket->mutable_aitoradio();

	aitoradioPacket->set_team_id(true);

    AIToRadio::Robot *r = aitoradioPacket->add_robots();

	RP::Vector disp(joy.getX(), joy.getY());

    //float disp_theta = 127 * joy.getZ()/1000;
	float disp_theta = 127 * ((float)joy.getZ()/((int)(1<<15)-1));

    //this crazy test is for us to determine we are pressing or not the direction button in the joystick
	//if not, we don't move the bot
    if( joy.getX() == 0 && joy.getY() == 0)
		disp = RP::Vector(0,0);

    disp.normalizeMe();

    r->set_displacement_x(disp.getX());
    r->set_displacement_y(disp.getY());
    r->set_displacement_theta(disp_theta);
    cout << "displacements:" << disp.getX() << "," << disp.getY() << endl;
    cout << "theta:" << disp_theta << endl;

	r->set_kick(joy.isPressed(KICK));
    r->set_drible(joy.isPressed(DRIBBLE));
    r->set_chip_kick(joy.isPressed(CHIP_KICK));
    r->set_id(current_bot);

	r->set_current_theta(90);

	joytoradioPacket->set_force_0(forces[current_bot][0]);
	joytoradioPacket->set_force_1(forces[current_bot][1]);
	joytoradioPacket->set_force_2(forces[current_bot][2]);

	joytoradioPacket->set_secret_attack(joy.isPressed(TATSUMAKI_SENPUU_KYAKU));

    joyToRadio.send(packet);

}

void changeBot(int sinal) {

	//if we are with the first or last bot, we do nothing
	if(sinal < 0 && current_bot == 0) {}
	else if(sinal > 0 && current_bot == MAX_BOT_INDEX) {}

	else {
		current_bot += sinal;
		current_motor = 0;
	}
}

void changeMotor(int sinal) {

	//if we are with the first or last bot, we do nothing
	if(sinal < 0 && current_motor == 0) {}
	else if(sinal > 0 && current_motor == MAX_MOTOR_INDEX) {}

	else current_motor += sinal;
}

void changeForce(int sinal) {

	forces[current_bot][current_motor] += sinal;
}

void processInput(Joystick &joy)
{

	cout << endl << endl << "Robot " << current_bot << endl;
	cout << "Motor: " << current_motor << " " << "Ajuste fino<" << forces[current_bot][0] << "," << forces[current_bot][1] << "," << forces[current_bot][2] << ">\n";

	vector<bool> buttons = joy.getButtonsPressed();

	joy.printStatus();

	if(buttons[INC_BOT])
		changeBot(INC);
	if(buttons[DEC_BOT])
		changeBot(DEC);
	if(buttons[INC_MOTOR])
		changeMotor(INC);
	if(buttons[DEC_MOTOR])
		changeMotor(DEC);
	if(buttons[INC_FORCE])
		changeForce(INC);
	if(buttons[DEC_FORCE])
		changeForce(DEC);

	sendToRadio(joy);
}

int openDevice(const char* device) {
	int fd = open(device, 0 );
	fcntl(fd, F_SETFL, O_RDONLY | O_NONBLOCK);

	return fd;
}

void printStatus(Joystick &joy)
{
	if(DEBUG)
		system("clear");
	vector<bool> buttons = joy.getButtonsPressed();
	for(unsigned int i = 0; i < buttons.size(); ++i)
		cout << buttons[i] << " | ";
	cout << endl;
	joy.printStatus();

}

int receiveInput(int device, Joystick &joy) {

	unsigned int len = 0;
	struct js_event msg;

	len = read(device, &msg, sizeof(msg));

	if (len == sizeof(msg)) { //read was succesfull

		if (msg.type == JS_EVENT_BUTTON) { // seems to be a key press
			joy.buttonInput(msg.number, msg.value);
		}

		if (msg.type == JS_EVENT_AXIS) {

			if (msg.number == 0) { // x axis
				joy.setX(msg.value);
			}

			if (msg.number == 1) { // y axis
				joy.setY(msg.value);
			}

			if (msg.number == 2) { // z axis
				joy.setZ(msg.value);
			}
		}

		printStatus(joy);
		processInput(joy);

	}


	return 0;

}

int main(int argc, char **argv)
{

	Joystick joy;
	int device = openDevice( "/dev/input/js0" );
	printf("Press <Enter> to open connection with client...\n");
	getchar();

    joyToRadio.open();

	if ( joy.isConfigured() )
		joy.loadConfig();

	while (true) {
		receiveInput(device, joy);
	}

    return 0;
}
