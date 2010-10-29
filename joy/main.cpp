#ifndef WIN32
#include <GL/glut.h>
#define GAMBI_INIT() glutInit(&argc, argv)
#else
#define GAMBI_INIT() ;;
#include <gl/glut.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "joy.h"
#include "rp_server.h"
#include "vector.h"

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



void sendToRadio(RoboPETServer joyToRadio) {

    RoboPET_WrapperPacket packet;
    AIToRadio *aitoradioPacket = packet.mutable_aitoradio();

    AIToRAdio::Robot *r = aitoradioPacket->add_robots();

    RP::Vector disp(global_joy.getX(), global_joy.getY());
    disp.normalizeMe();

    float disp_theta = 360 * global_joy/1000;

    r->set_displacement_x(disp.getX());
    r->set_displacement_y(disp.getY());
    r->set_displacement_theta(disp_theta);

    r->set_kick(0);
    r->set_drible(0);
    r->set_id(current_bot);
    r->set_current_theta(0);

    aitoradio.send(packet);

}

void JoystickFunc(unsigned int mask, int x, int y, int z)
{

		global_joy.receiveInput(mask,x,y,z);

		vector<int> vectorzenyo = global_joy.getAll();
		for(unsigned int i = 0; i < vectorzenyo.size(); ++i)
			cout << vectorzenyo[i] << " | ";
		cout << endl;
		global_joy.printStatus();

        sendToRAdio(joyToRadio);
}

int main(int argc, char **argv)
{

    joyToRadio.open();


	//incializa a glut para funcionar no windows e no linux
    GAMBI_INIT();

	//so para mostrar na tela
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    //tela
    glutCreateWindow("RoboPET Joy Control");
    //registra na glut os callbacks
    glutDisplayFunc(RenderScene);
    //chama de 100 em 100 ms
    glutJoystickFunc(JoystickFunc,100);
    SetupRC();
    glutMainLoop();

    return 0;
}
