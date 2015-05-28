#ifdef WIN32
#include <windows.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include "hidapi.h"

#define MAX_STR 255

short shortify(unsigned char* array, int offset){
	return (short)(((short)array[offset]<<8) | array[offset+1]);
}

int main(int argc, char* argv[])
{
	FILE *ofp;
	int res;
	unsigned char buf[65];
	wchar_t wstr[MAX_STR];
	hid_device *handle;
	int i;
	int j;
	int k;
	int time=0;
	short x[50];
	short y[50];
	short z[50];
	short accels[3];
	short MAFaccels[50];
	short FIRaccels[50];
	// Initialize the hidapi library
	res = hid_init();

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	handle = hid_open(0x4d8, 0x3f, NULL);

	// Read the Manufacturer String
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	wprintf(L"Manufacturer String: %s\n", wstr);

	// Read the Product String
	res = hid_get_product_string(handle, wstr, MAX_STR);
	wprintf(L"Product String: %s\n", wstr);

	// Read the Serial Number String
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	wprintf(L"Serial Number String: (%d) %s\n", wstr[0], wstr);

	// Read Indexed String 1
	res = hid_get_indexed_string(handle, 1, wstr, MAX_STR);
	wprintf(L"Indexed String 1: %s\n", wstr);

	// Toggle LED (cmd 0x80). The first byte is the report number (0x0).
	buf[0] = 0x0;
	buf[1] = 0x80;
	char message[25];
	printf("Input: \n");
	scanf("%s",message);
	for (j=2; j<10; j++){
		buf[j] = message[j-2];
	}
	res = hid_write(handle, buf, 65);

	// Request state (cmd 0x81). The first byte is the report number (0x0).
	while(time<50){
		buf[0] = 0x0;
		buf[1] = 0x81;

		// Print out the returned buffer.
		for (i = 0; i < 6; i++)
			printf("buf[%d]: %d\n", i, buf[i]);
	
		res = hid_write(handle, buf, 65);

		// Read requested state
		res = hid_read(handle, buf, 65);

	// Print out the returned buffer.
		if(buf[1]){
			for(k=0; k<1; k++){
				accels[k] = shortify(buf,2*k+2);
			}
			MAFaccels[time] = shortify(buf,4);
			FIRaccels[time] = shortify(buf,6);
			
			z[time] = accels[0];
			time++;
		}
	// Finalize the hidapi library
		res = hid_exit();
	}

	ofp = fopen("accels.txt", "w");

	for (i=0; i<50; i++) {

		fprintf(ofp,"%d, %d, %d, %d, %d\r\n",x[i],y[i],z[i],MAFaccels[i],FIRaccels[i]);

	}

fclose(ofp);

	return 0;
}