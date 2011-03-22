CC = g++
CFLAGS = -Wall
GLUTFLAGS = #-lglut -lGL -lGLU -lX11 -lm

INCLUDE_PATHS = -I$(ROBOPET_PATH) -I$(COMMUNICATION_H) -I$(SOCKETS_PATH)

ROBOPET_PATH = ../../lib
ROBOPET_LIB = $(ROBOPET_PATH)/robopet.a

COMMUNICATION_PATH = ../../communication
COMMUNICATION_H = $(COMMUNICATION_PATH)/packets
COMMUNICATION_LIB = $(COMMUNICATION_PATH)/communication.a

SOCKETS_PATH = $(COMMUNICATION_PATH)/socket

OBJECTS = joy.o stringEx.o fileUtils.o

.PHONY : all clean

all: joy

joy: main.cpp $(OBJECTS) $(ROBOPET_LIB) $(COMMUNICATION_LIB)
	$(CC) -o $@ $^ $(CFLAGS) $(GLUTFLAGS) $(INCLUDE_PATHS) `pkg-config --cflags --libs protobuf`

%.o: %.cpp %.h
	$(CC) -c -o $@ $< $(CFLAGS)

