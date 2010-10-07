#ifndef __PETUSB_H
#define __PETUSB_H

#include <iostream>
#include <cstdlib>
#include <libusb-1.0/libusb.h>

using namespace std;

/*
Endpoint Definition (by eduardo): interface comum de entrada e saída
*/

/*usbInitializeDevice : intializes libusb-1.0 and our device according to the vendorId and productId
						returns a handle to the device or NULL if we couldn't open the device
*/
libusb_device_handle * usbInitializeDevice( int vendorId, int productId, libusb_context *ctx );


/*int usbSendData: sends data to our device.
					returns 1 if successful or 0 if error
*/
int usbSendData( unsigned char *data, libusb_device_handle *dev_handle, int endpoint_add, int num_bytes );

/*int usbReceiveData: receives data from our device.
					  returns 1 if successful or 0 if error
*/
int usbReceiveData( unsigned char *data, libusb_device_handle *dev_handle, int endpoint_add, int num_bytes );


/*int usbClosingDevice: closes our device and libusb-1.0
						returns 1 if successful or 0
*/
int usbClosingDevice( libusb_context *ctx, libusb_device_handle *dev_handle );



#endif
