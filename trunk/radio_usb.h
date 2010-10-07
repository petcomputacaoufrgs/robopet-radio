#ifndef _RADIOUSB_H_
#define _RADIOUSB_H_

#include <iostream>
#include <cstdlib>
#include <libusb-1.0/libusb.h>

using namespace std;


//these constants are defined for our PIC device
#define ENDPOINT_ADDRESS 1
#define VENDOR_ID 		 0x04d8
#define PRODUCT_ID 		 0x0042


class RadioUSB
{
	public:
		RadioUSB();
		~RadioUSB();		
		/*usbInitializeDevice : intializes libusb-1.0 and our device according to the vendorId and productId
								returns a handle to the device or NULL if we couldn't open the device
		*/
		libusb_device_handle * usbInitializeDevice();

		/*int usbSendData: sends data to our device.
							returns 1 if successful or 0 if error
		*/
		int usbSendData( unsigned char *data, int num_bytes );

		/*int usbClosingDevice: closes our device and libusb-1.0
								returns 1 if successful or 0
		*/
		int usbClosingDevice();		

	private:
		libusb_device_handle *_dev_handle; //a device handle
		libusb_context *_ctx; //a libusb session
		int _vendorId, _productId, _endpoint_address;

		//Endpoint Definition (by eduardo): interface comum de entrada e saída
};



#endif
