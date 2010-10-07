#include "petusb.h"

//usbInitializeDevice : intializes libusb-1.0 and our device according to the vendorId and productId
//						returns a handle to the device or NULL if we couldn't open the device
libusb_device_handle * usbInitializeDevice( int vendorId, int productId, libusb_context *ctx )
{
	libusb_device_handle *dev_handle; //a device handle
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt; //holding number of devices in list
	int r; //for return values

	/* Initializing */
	r = libusb_init(&ctx); //initialize the library for the session we just declared
	if(r < 0) {
		cout<<"Init Error "<<r<<endl; //there was an error
		return NULL;
	}

	/* Set debug level */
	libusb_set_debug(ctx, 3); //set verbosity level to 3, as suggested in the documentation

    /* Get list of devices */
	cnt = libusb_get_device_list(ctx, &devs); //get the list of devices
	if(cnt < 0) {
		cout<<"Get Device Error"<<endl; //there was an error
		return NULL;
	}
	cout << cnt << " Devices in list." << endl;

    /* Opening device */
    //0x04d8, 0x0042
	dev_handle = libusb_open_device_with_vid_pid(ctx, vendorId, productId); //these are vendorID and productID founded

	if(!dev_handle) {
		cout<<"Cannot open device"<<endl;
	}
	else {
		cout<<"Device Opened"<<endl;
    }

	libusb_free_device_list(devs, 1); //free the list, unref the devices in it


    //Determine if a kernel driver is active on an interface.
    //If a kernel driver is active, you cannot claim the interface, and libusb will be unable to perform I/O
    //So, we want it to be 0!
	if(libusb_kernel_driver_active(dev_handle, 0) == 1) {
		cout<<"Kernel Driver Active"<<endl;

		//if a kernel drive is active, we've got to detach it
        //Detach a kernel driver from an interface.
        //If successful, you will then be able to claim the interface and perform I/O.
		if(libusb_detach_kernel_driver(dev_handle, 0) == 0) //detach it
			cout<<"Kernel Driver Detached!"<<endl;
	}


	r = libusb_claim_interface(dev_handle, 0); //claim interface 0 (the first) of device (mine had jsut 1)
	if(r < 0) {
		cout<<"Cannot Claim Interface"<<endl;
		return NULL;
	}
	cout<<"Claimed Interface"<<endl;

	return dev_handle;
}

//int usbSendData: sends data to our device.
//					returns 1 if successful or 0 if error
int usbSendData( unsigned char *data, libusb_device_handle *dev_handle, int endpoint_add, int num_bytes )
{
	int actual; //used to find out how many bytes were written
	int r; //for return values

	r = libusb_bulk_transfer(dev_handle, (endpoint_add | LIBUSB_ENDPOINT_OUT), data, num_bytes, &actual, 0);

	cout << "Data->" << data << "<-" << endl; //just to see the data we want to write
	cout << "Writing Data..." << endl;
	if(r == 0 && actual == num_bytes) //we wrote the 64 bytes successfully
	{
		cout<<"Writing Successful!"<<endl;
		return 1;
	}
	else
	{
		cout<<"Write Error"<<endl;
		return 0;
	}
}

//int usbReceiveData: receives data from our device.
//					  returns 1 if successful or 0 if error
int usbReceiveData( unsigned char *data_receive, libusb_device_handle *dev_handle, int endpoint_add, int num_bytes )
{
	int actual; //used to find out how many bytes were written
	int r; //for return values

	r = libusb_bulk_transfer(dev_handle, (endpoint_add | LIBUSB_ENDPOINT_IN), data_receive, num_bytes, &actual, 0);


	if(r == 0 && actual == num_bytes) //we read the 64 bytes successfully
	{
		cout<<"Reading Successful!"<<endl;
		return 1;
	}
	else
	{
		cout<<"Read Error"<<endl;
		return 0;
	}
}


//int usbClosingDevice: closes our device and libusb-1.0
//						returns 1 if successful or 0
int usbClosingDevice( libusb_context *ctx, libusb_device_handle *dev_handle )
{
	int r; //for return values

	r = libusb_release_interface(dev_handle, 0); //release the claimed interface
	if(r != 0) {
		cout<<"Cannot Release Interface"<<endl;
		return 0;
	}
	cout<<"Released Interface"<<endl;
	libusb_close(dev_handle); //close the device we opened
	libusb_exit(ctx); //needs to be called to end the

	return 1;
}



