#include <iostream>
#include <cstdlib>
#include <libusb-1.0/libusb.h>
#include "petusb.h"

//these constants are defined for our PIC device
#define BYTES_TO_WRITE 	 64
#define BYTES_TO_RECEIVE 64
#define ENDPOINT_ADDRESS 1
#define VENDOR_ID 		 0x04d8
#define PRODUCT_ID 		 0x0042

#define TOGGLE_LED		128
#define SWITCH_STATUS	129
#define LED_STATUS		130


/*
Send:
buf[0] = 128 //toggle led state
	Receive:
	buf[0] = 128

Send:
buf[0] = 129 //asks for the switch state
	Receive:
	buf[0] = 129
	buf[1] = 1 ou 0

Send:
buf[0] = 130 //asks for the led state
	Receive:
	buf[0] = 130
	buf[1] = 1 ou 0
*/


int main(int argc, char *argv[]) {
	libusb_device_handle *dev_handle; //a device handle
	libusb_context *ctx = NULL; //a libusb session
	unsigned char data_send[BYTES_TO_WRITE] = {0}; //data to write
	unsigned char data_receive[BYTES_TO_RECEIVE] = {0}; //data received

	/* Initializes, Set debug level, Get list of devices, Open device, Claim Interface  */
	dev_handle = usbInitializeDevice( VENDOR_ID, PRODUCT_ID, ctx );
	if( dev_handle != NULL) {
		data_send[0] = atoi(argv[1]);

		if(data_send[0] == TOGGLE_LED) {
			cout<<"Sending Data\n";
			cout<<"Toggle led!!"<<endl;
			if( usbSendData( data_send, dev_handle, ENDPOINT_ADDRESS, BYTES_TO_WRITE ) != 1 ) {
				return 1;
			}
		}

		else {
			if(data_send[0] == SWITCH_STATUS || data_send[0] == LED_STATUS) {
				if( usbSendData( data_send, dev_handle, ENDPOINT_ADDRESS, BYTES_TO_WRITE ) != 1 ) {
					return 1;
				}
				if( usbReceiveData( data_receive, dev_handle, ENDPOINT_ADDRESS, BYTES_TO_RECEIVE ) != 1 ) {
					return 1;
				}
				cout<<"Received Data:\n"<<endl;
				cout<<"data_received[0]: "<<(int)data_receive[0]<<", data_received[1]: "<<(int)data_receive[1]<<endl;
			}
		}

		/* Closing Device */
		if( usbClosingDevice( ctx, dev_handle ) != 1 )
			return 1;
	}

	else {
		return 1;
	}


	return 0;
}

