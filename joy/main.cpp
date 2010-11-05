#include <GL/glut.h>

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "joy.h"
#include "rp_server.h"
#include "vector.h"

#define INC 1
#define DEC -1

Joystick global_joy;

RoboPETServer joyToRadio(PORT_AI_TO_RADIO, IP_AI_TO_RADIO);
int current_bot = 0;


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

double calcCurrentTheta() {

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

    AIToRadio *aitoradioPacket = packet.mutable_aitoradio();

    AIToRadio::Robot *r = aitoradioPacket->add_robots();

    RP::Vector disp(global_joy.getX(), global_joy.getY());
    disp.normalizeMe();

    float disp_theta = 360 * global_joy.getZ()/1000;

    r->set_displacement_x(0);
	//this crazy test is for us to determine we are pressing or not the direction button in the joystick
	//if not, we don't move the bot
    r->set_displacement_y((abs(global_joy.getX()) == 1000 ||
						   abs(global_joy.getY()) == 1000  ) ?	1 : 0);
    r->set_displacement_theta(disp_theta);

	r->set_kick(global_joy.isPressed(KICK));
    r->set_drible(global_joy.isPressed(PASS));
    r->set_id(current_bot);

	r->set_current_theta(calcCurrentTheta());

    joyToRadio.send(packet);

}

void changeBot(int sinal) {

	//if we are with the first or last bot, we do nothing
	if(sinal < 0 && current_bot == 0) {}
	else if(sinal > 0 && current_bot == 4) {}

	else current_bot += sinal;
}

void JoystickFunc(unsigned int mask, int x, int y, int z)
{

	cout << endl << endl << "Robot " << current_bot << endl;
	global_joy.receiveInput(mask,x,y,z);

	vector<bool> buttons = global_joy.getButtonsPressed();
	for(unsigned int i = 0; i < buttons.size(); ++i)
		cout << buttons[i] << " | ";
	cout << endl;
	global_joy.printStatus();

	//SHORYUKEN and BRAKE are the names of the buttons -> joy.conf file
	if(buttons[SHORYUKEN])
		changeBot(INC);
	if(buttons[BRAKE])
		changeBot(DEC);

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
