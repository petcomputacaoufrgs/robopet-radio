#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"

RoboCupSSLServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboCupSSLClient aitosim(PORT_AI_TO_SIM, IP_SIM_TO_TRACKER);

int DEBUG = 1;

void receive()
{
	SSL_WrapperPacket packet;
	if (aitosim.receive(packet)) {
		printf("----------------------------\n");
		printf("TestClient Received ");

		if (packet.has_aitosim())
		{
			printf("AI-To-SIM!\n");
		}

/*
		if (packet.has_trackertoai())
		{
			if(DEBUG)
			{
				printf("Tracker-To-AI!\n");
				printf("Ball: <%d, %d>\n", packet.trackertoai().ball().x(),
				 							packet.trackertoai().ball().y());
			
			
				for(int i=0; i<packet.trackertoai().robots_blue_size(); i++)
					printf("Blue Robot[%d]: <%d, %d>\n", i,
											packet.trackertoai().robots_blue(i).x(),
											packet.trackertoai().robots_blue(i).y());
				for(int i=0; i<packet.trackertoai().robots_yellow_size(); i++)
					printf("Yellow Robot[%d]: <%d, %d>\n", i,
											packet.trackertoai().robots_yellow(i).x(),
											packet.trackertoai().robots_yellow(i).y());
			}
			
			_ball.setCurrentPosition(Point(packet.trackertoai().ball().x(),
				 						   packet.trackertoai().ball().y()));
				 						   
			for(int i=0; i<packet.trackertoai().robots_blue_size(); i++)
				_players[i].setCurrentPosition(Point(packet.trackertoai().robots_blue(i).x(),
												     packet.trackertoai().robots_blue(i).y()));
		}//*/
	}
}

void send()
{
	 SSL_WrapperPacket packet;

	 SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	 BallSim *b = simtotrackerPacket->mutable_ball();
	 
	 b->set_x(77);
	 b->set_y(666);
	 
	 simtotracker.send(packet);
}

int main()
{
	printf("Simulator Running!\n");
	
	aitosim.open(false);
	
	printf("Press <Enter> to open connection with client...\n");
	getchar();
	simtotracker.open();
	
	while(1) {
		receive();
		send();
	}
}
