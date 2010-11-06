#include <GL/glut.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "joy.h"
#include "rp_server.h"
#include "vector.h"

#define INC 1
#define DEC -1

#define MAX_BOT 5
#define MAX_MOTOR 3
#define MAX_BOT_INDEX MAX_BOT - 1
#define MAX_MOTOR_INDEX MAX_MOTOR - 1

Joystick global_joy;

RoboPETServer joyToRadio(PORT_JOY_TO_RADIO, IP_JOY_TO_RADIO);
int current_bot = 0;
int current_motor = 0;
int forces[5][3] = {{0}};


void SetupRC(void)
{
		if (global_joy.isConfigured())
		{
			global_joy.loadConfig();
		}

	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void RenderScene(void)
{
     glutSwapBuffers();
}

double calcAnalogicTheta() {

	double theta=0;

	if(global_joy.getY() == 1000) {
		theta = 90 - 45 * global_joy.getX()/1000;
	}
	else if (global_joy.getX() == -1000) {
		theta = 180 - 45 * global_joy.getY()/1000;
	}
	else if (global_joy.getY() == -1000) {
		theta = 270 + 45 * global_joy.getX()/1000;
	}
	else if (global_joy.getX() == 1000) {
		theta = 360 + 45 * global_joy.getY()/1000;
	}

	if (theta >= 360) {
		theta -= 360;
	}

	return theta;
}

void sendToRadio() {

    RoboPET_WrapperPacket packet;

    JoyToRadio *joytoradioPacket = packet.mutable_joytoradio();
	AIToRadio *aitoradioPacket = joytoradioPacket->mutable_aitoradio();

    AIToRadio::Robot *r = aitoradioPacket->add_robots();

    RP::Vector disp(global_joy.getX(), global_joy.getY());
    disp.normalizeMe();

    float disp_theta = 360 * global_joy.getZ()/1000;
    float analogic_angle = calcAnalogicTheta();

    r->set_displacement_x(cos(analogic_angle*3.1415/180));
	//this crazy test is for us to determine we are pressing or not the direction button in the joystick
	//if not, we don't move the bot
    r->set_displacement_y(sin(analogic_angle*3.1415/180));
    r->set_displacement_theta(disp_theta);

	r->set_kick(global_joy.isPressed(KICK));
    r->set_drible(global_joy.isPressed(DRIBBLE));
    r->set_chip_kick(global_joy.isPressed(CHIP_KICK));
    r->set_id(current_bot);

	r->set_current_theta(0);
	
	joytoradioPacket->set_force_0(forces[current_bot][0]);
	joytoradioPacket->set_force_1(forces[current_bot][1]);
	joytoradioPacket->set_force_2(forces[current_bot][2]);
	joytoradioPacket->set_secret_attack(global_joy.isPressed(TATSUMAKI_SENPUU_KYAKU));

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

void JoystickFunc(unsigned int mask, int x, int y, int z)
{

	cout << endl << endl << "Robot " << current_bot << endl;
	cout << "Motor: " << current_motor << "Ajuste fino<" << forces[current_bot][0] << "," << forces[current_bot][1] << "," << forces[current_bot][2] << ">\n";
		
	global_joy.receiveInput(mask,x,y,z);

	vector<bool> buttons = global_joy.getButtonsPressed();
	for(unsigned int i = 0; i < buttons.size(); ++i)
		cout << buttons[i] << " | ";
	cout << endl;
	global_joy.printStatus();

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
}

void idleFunc() {
	sendToRadio();
}

int main(int argc, char **argv)
{

	printf("Press <Enter> to open connection with client...\n");
	getchar();

    joyToRadio.open();

	glutInit(&argc, argv);

	//so para mostrar na tela
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    //tela
    glutCreateWindow("RoboPET Joy Control");
    //registra na glut os callbacks
    glutDisplayFunc(RenderScene);
    //chama de 100 em 100 ms
    glutJoystickFunc(JoystickFunc,100);

	glutIdleFunc(idleFunc);

    SetupRC();
    glutMainLoop();

    return 0;
}
