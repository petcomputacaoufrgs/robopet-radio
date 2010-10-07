CC=g++
CFLAGS=-Wall -lusb-1.0
FILE=petusb
MAIN=communication_usb.cpp
OBJECTS=$(FILE).o

all: communication_usb

communication_usb: $(MAIN) $(OBJECTS)
	$(CC) $^ -o $@ $(CFLAGS)

%.o: %.cpp
	$(CC) -c -o $@ $*.cpp $(CFLAGS)
