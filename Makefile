CC = colorgcc
FLAGS = -Wall -I$(AI_PATH) -I$(PROTOBUF_H) -I$(SOCKETS_PATH) -lstdc++ -lprotobuf

AI_PATH = ../IA/lib
AI_OBJECTS = $(AI_PATH)/point.o $(AI_PATH)/vector.o $(AI_PATH)/movingObject.o

PROTOBUF_PATH = ../Communication/SSL
PROTOBUF_H = $(PROTOBUF_PATH)/packets

PROTOBUF_FILES =    message_gui_to_ai.pb.o \
					message_ai_to_gui.pb.o \
					message_ai_to_radio.pb.o \
					messages_robocup_ssl_detection.pb.o \
					messages_robocup_ssl_geometry.pb.o \
					messages_robocup_ssl_refbox_log.pb.o \
					messages_robocup_ssl_wrapper.pb.o \
					message_tracker_to_ai.pb.o \
					message_sim_to_tracker.pb.o \
					message_ai_to_sim.pb.o \
					ssl_client.o \
					ssl_server.o \
					udp_netraw.o 

SOCKETS_PATH = $(PROTOBUF_PATH)/socket

PROTOBUF_O = $(PROTOBUF_FILES:%=$(PROTOBUF_PATH)/%)

all: simulator

simulator: main.cpp $(AI_OBJECTS) $(PROTOBUF_O)
	@echo $@
	@$(CC) -o $@ $^ $(FLAGS)
	
