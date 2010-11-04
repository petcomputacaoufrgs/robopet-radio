#ifndef _RADIOUSB_CPP_
#define _RADIOUSB_CPP_

#include "radio_usb.h"

RadioUSB::RadioUSB(bool realRadio)
{
    _dummy = !realRadio;
	_ctx = NULL;
	_dev_handle = NULL;

	 _vendorId = VENDOR_ID;
	 _productId = PRODUCT_ID;
	 _endpoint_address = ENDPOINT_ADDRESS;
};

RadioUSB::~RadioUSB() {};

//usbInitializeDevice : intializes libusb-1.0 and our device according to the vendorId and productId
//						returns a handle to the device or NULL if we couldn't open the device
libusb_device_handle* RadioUSB::usbInitializeDevice()
{
    if (!_dummy) {

        libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	    ssize_t cnt; //holding number of devices in list
	    int r; //for return values

	    /* Initializing */
	    r = libusb_init(&_ctx); //initialize the library for the session we just declared
	    if(r < 0) {
		    printf("Init Error %d\n",r); //there was an error
		    return NULL;
	    }

	    /* Set debug level */
	    libusb_set_debug(_ctx, 3); //set verbosity level to 3, as suggested in the documentation

        /* Get list of devices */
	    cnt = libusb_get_device_list(_ctx, &devs); //get the list of devices
	    if(cnt < 0) {
		    printf("Get Device Error\n"); //there was an error
		    return NULL;
	    }

	    printf("%d devices in list.\n",cnt);

        /* Opening device */
        //0x04d8, 0x0042
	    _dev_handle = libusb_open_device_with_vid_pid(_ctx, VENDOR_ID, PRODUCT_ID); //these are vendorID and productID found

	    if(!_dev_handle) {
		    printf("Cannot open device\n");
	    }
	    else {
		    printf("Device Opened\n");
        }

	    libusb_free_device_list(devs, 1); //free the list, unref the devices in it


        //Determine if a kernel driver is active on an interface.
        //If a kernel driver is active, you cannot claim the interface, and libusb will be unable to perform I/O
        //So, we want it to be 0!
	    if(libusb_kernel_driver_active(_dev_handle, 0) == 1) {
		    printf("Kernel Driver Active\n");

		    //if a kernel drive is active, we've got to detach it
            //Detach a kernel driver from an interface.
            //If successful, you will then be able to claim the interface and perform I/O.
		    if(libusb_detach_kernel_driver(_dev_handle, 0) == 0) //detach it
			    printf("Kernel Driver Detached!\n");
	    }


	    r = libusb_claim_interface(_dev_handle, 0); //claim interface 0 (the first) of device (mine had jsut 1)
	    if(r < 0) {
		    printf("Cannot Claim Interface\n");
		    return NULL;
	    }

	    printf("Claimed Interface\n");

    }

	return _dev_handle;
}

//int usbSendData: sends data to our device.
//					returns 1 if successful or 0 if error
int RadioUSB::usbSendData( unsigned char *data, int num_bytes )
{
    if (! _dummy) {

        int actual; //used to find out how many bytes were written
        int r; //for return values

        for(int i = 0; i < num_bytes; i++)
	        printf("\nData[%d]: %d\n", i, data[i]); //just to see the data we want to write
        r = libusb_bulk_transfer(_dev_handle, (_endpoint_address | LIBUSB_ENDPOINT_OUT), data, num_bytes, &actual, 0);

        printf("Writing Data...\n");
        if(r == 0 && actual == num_bytes) //we wrote the bytes successfully
        {
	        printf("Writing Successful!\n");
	        return 1;
        }
        else
        {
	        printf("Write Error\n");
	        return 0;
        }

    }

    printf("Writing to mock radio\n");
    return 1;

}

//int usbClosingDevice: closes our device and libusb-1.0
//						returns 1 if successful or 0
int RadioUSB::usbClosingDevice()
{
    if (! _dummy ) {

        int r; //for return values

        r = libusb_release_interface(_dev_handle, 0); //release the claimed interface
        if(r != 0) {
	        printf("Cannot Release Interface\n");
	        return 0;
        }

        printf("Released Interface\n");
        libusb_close(_dev_handle); //close the device we opened
        libusb_exit(_ctx); //needs to be called to end the

    }

	return 1;
}

#endif

