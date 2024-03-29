#ifndef __JOY_H_
#define __JOY_H_

#include "stringEx.h"
#include "fileUtils.h"

#define DEFAULT_JOY "joy"

enum {
		BUTTON_1 = 0,
		BUTTON_2,
		BUTTON_3,
		BUTTON_4,
		BUTTON_5,
		BUTTON_6,
		BUTTON_7,
		BUTTON_8,
		BUTTON_9,
		BUTTON_10,
		BUTTONS_TOTAL
};

static string buttonStrings[BUTTONS_TOTAL] = { "BUTTON_1","BUTTON_2","BUTTON_3",
											 "BUTTON_4","BUTTON_5","BUTTON_6",
							 				 "BUTTON_7","BUTTON_8","BUTTON_9",
											 "BUTTON_10"};

enum {
		KICK = 0,
		DRIBBLE,
		CHIP_KICK,
		TATSUMAKI_SENPUU_KYAKU,
		DEC_BOT,
		DEC_FORCE,
		INC_BOT,
		INC_FORCE,
		DEC_MOTOR,
		INC_MOTOR,
		ACTIONS_TOTAL
};

static string actionStrings[ACTIONS_TOTAL] = { "KICK", "DRIBBLE", "CHIP_KICK",
											   "TATSUMAKI_SENPUU_KYAKU", "DEC_BOT", "DEC_FORCE", "INC_BOT", "INC_FORCE", "DEC_MOTOR", "INC_MOTOR"
		 };

class Joystick {

		public:
			Joystick();
			Joystick(string joyname);

			~Joystick();
			bool isConfigured();
			void saveConfig();
			void loadConfig();
			void configureButtons();
			void mapButton(int action, int button);
			void printStatus();

			void buttonInput(int number, int pressed);
			void axisInput(int x, int y, int z);

			bool keyPressed();
			bool axesMoved();

			vector<int>  getAxes();
			bool isPressed(int button);
			vector<bool> getButtonsPressed();
			vector<int> getAll();

			//getters
			int getX();
			int getY();
			int getZ();
			int getMask();

			//setters
			void setX(int newx);
			void setY(int newy);
			void setZ(int newz);

			void setRange(int start,int end);

		private:
			string name;
			int x,y,z;

			//cada ação está associada ao seu botão respectivo
			int buttonMapping[ACTIONS_TOTAL];
			//array das ações atuais;
			bool currentActions[ACTIONS_TOTAL];

			//Métodos Privados
			//retorna o índice de uma ação dada como uma string
			int getActionIndex(string action);
			//retorna o índice de um botão dado como uma string
			int getButtonIndex(string button);

			void printActions();

			int rangeStart, rangeEnd;

};

#endif
