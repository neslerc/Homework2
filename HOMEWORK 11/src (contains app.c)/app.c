/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "system_definitions.h"
#include "app.h"
#include "i2c_display.h"
#include "i2c_master_int.h"
#include "accel.h"



// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************


// lookup table for all of the ascii characters
static const char ASCII[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00} // 20  (space)
    ,
    {0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
    ,
    {0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
    ,
    {0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
    ,
    {0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
    ,
    {0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
    ,
    {0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
    ,
    {0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
    ,
    {0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
    ,
    {0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
    ,
    {0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
    ,
    {0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
    ,
    {0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
    ,
    {0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
    ,
    {0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
    ,
    {0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
    ,
    {0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
    ,
    {0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
    ,
    {0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
    ,
    {0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
    ,
    {0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
    ,
    {0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
    ,
    {0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
    ,
    {0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
    ,
    {0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
    ,
    {0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
    ,
    {0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
    ,
    {0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
    ,
    {0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
    ,
    {0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
    ,
    {0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
    ,
    {0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
    ,
    {0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
    ,
    {0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
    ,
    {0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
    ,
    {0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
    ,
    {0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
    ,
    {0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
    ,
    {0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
    ,
    {0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
    ,
    {0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
    ,
    {0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
    ,
    {0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
    ,
    {0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
    ,
    {0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
    ,
    {0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
    ,
    {0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
    ,
    {0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
    ,
    {0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
    ,
    {0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
    ,
    {0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
    ,
    {0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
    ,
    {0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
    ,
    {0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
    ,
    {0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
    ,
    {0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
    ,
    {0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
    ,
    {0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
    ,
    {0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
    ,
    {0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
    ,
    {0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
    ,
    {0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
    ,
    {0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
    ,
    {0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
    ,
    {0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
    ,
    {0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
    ,
    {0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
    ,
    {0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
    ,
    {0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
    ,
    {0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
    ,
    {0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
    ,
    {0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
    ,
    {0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
    ,
    {0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
    ,
    {0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j
    ,
    {0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
    ,
    {0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
    ,
    {0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
    ,
    {0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
    ,
    {0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
    ,
    {0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
    ,
    {0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
    ,
    {0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
    ,
    {0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
    ,
    {0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
    ,
    {0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
    ,
    {0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
    ,
    {0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
    ,
    {0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
    ,
    {0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
    ,
    {0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
    ,
    {0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
    ,
    {0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
    ,
    {0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
    ,
    {0x10, 0x08, 0x08, 0x10, 0x08} // 7e ?
    ,
    {0x00, 0x06, 0x09, 0x09, 0x06} // 7f ?
}; // end char ASCII[96][5]



/* Recieve data buffer */
uint8_t receiveDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

/* Transmit data buffer */
uint8_t  transmitDataBuffer[64] APP_MAKE_BUFFER_DMA_READY;

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

USB_DEVICE_HID_EVENT_RESPONSE APP_USBDeviceHIDEventHandler
(
    USB_DEVICE_HID_INDEX iHID,
    USB_DEVICE_HID_EVENT event,
    void * eventData,
    uintptr_t userData
)
{
    USB_DEVICE_HID_EVENT_DATA_REPORT_SENT * reportSent;
    USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED * reportReceived;

    /* Check type of event */
    switch (event)
    {
        case USB_DEVICE_HID_EVENT_REPORT_SENT:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_SENT
             * pointer type containing details about the report that was
             * sent. */
            reportSent = (USB_DEVICE_HID_EVENT_DATA_REPORT_SENT *) eventData;
            if(reportSent->handle == appData.txTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataTransmitted = true;
            }
            
            break;

        case USB_DEVICE_HID_EVENT_REPORT_RECEIVED:

            /* The eventData parameter will be USB_DEVICE_HID_EVENT_REPORT_RECEIVED
             * pointer type containing details about the report that was
             * received. */

            reportReceived = (USB_DEVICE_HID_EVENT_DATA_REPORT_RECEIVED *) eventData;
            if(reportReceived->handle == appData.rxTransferHandle )
            {
                // Transfer progressed.
                appData.hidDataReceived = true;
            }
          
            break;

        case USB_DEVICE_HID_EVENT_SET_IDLE:

            /* For now we just accept this request as is. We acknowledge
             * this request using the USB_DEVICE_HID_ControlStatus()
             * function with a USB_DEVICE_CONTROL_STATUS_OK flag */

            USB_DEVICE_ControlStatus(appData.usbDevHandle, USB_DEVICE_CONTROL_STATUS_OK);

            /* Save Idle rate recieved from Host */
            appData.idleRate = ((USB_DEVICE_HID_EVENT_DATA_SET_IDLE*)eventData)->duration;
            break;

        case USB_DEVICE_HID_EVENT_GET_IDLE:

            /* Host is requesting for Idle rate. Now send the Idle rate */
            USB_DEVICE_ControlSend(appData.usbDevHandle, & (appData.idleRate),1);

            /* On successfully reciveing Idle rate, the Host would acknowledge back with a
               Zero Length packet. The HID function drvier returns an event
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT to the application upon
               receiving this Zero Length packet from Host.
               USB_DEVICE_HID_EVENT_CONTROL_TRANSFER_DATA_SENT event indicates this control transfer
               event is complete */

            break;
        default:
            // Nothing to do.
            break;
    }
    return USB_DEVICE_HID_EVENT_RESPONSE_NONE;
}

void APP_USBDeviceEventHandler(USB_DEVICE_EVENT event, void * eventData, uintptr_t context)
{
    switch(event)
    {
        case USB_DEVICE_EVENT_RESET:
        case USB_DEVICE_EVENT_DECONFIGURED:

            /* Host has de configured the device or a bus reset has happened.
             * Device layer is going to de-initialize all function drivers.
             * Hence close handles to all function drivers (Only if they are
             * opened previously. */

            BSP_LEDOn  (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            BSP_LEDOff (APP_USB_LED_3);
            appData.deviceConfigured = false;
            appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            break;

        case USB_DEVICE_EVENT_CONFIGURED:
            /* Set the flag indicating device is configured. */
            appData.deviceConfigured = true;

            /* Save the other details for later use. */
            appData.configurationValue = ((USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData)->configurationValue;

            /* Register application HID event handler */
            USB_DEVICE_HID_EventHandlerSet(USB_DEVICE_HID_INDEX_0, APP_USBDeviceHIDEventHandler, (uintptr_t)&appData);

            /* Update the LEDs */
            BSP_LEDOff (APP_USB_LED_1);
            BSP_LEDOff (APP_USB_LED_2);
            BSP_LEDOn  (APP_USB_LED_3);

            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch on green and orange, switch off red */
            BSP_LEDOff (APP_USB_LED_1);
            BSP_LEDOn  (APP_USB_LED_2);
            BSP_LEDOn  (APP_USB_LED_3);
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */

            USB_DEVICE_Attach (appData.usbDevHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available */
            USB_DEVICE_Detach(appData.usbDevHandle);
            break;

        /* These events are not used in this demo */
        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    appData.usbDevHandle = USB_DEVICE_HANDLE_INVALID;
    appData.deviceConfigured = false;
    appData.txTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.rxTransferHandle = USB_DEVICE_HID_TRANSFER_HANDLE_INVALID;
    appData.hidDataReceived = false;
    appData.hidDataTransmitted = true;
    appData.receiveDataBuffer = &receiveDataBuffer[0];
    appData.transmitDataBuffer = &transmitDataBuffer[0];
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks (void )
{
    char message[25];
    char inputfrompc[25];
    int i;
    short accels[3]; // accelerations for the 3 axes
    short accelsMAF;
    short accelsFIR;
    int buf1=0;
    int buf2=0;
    int buf3=0;
    int buf4=0;
    int buf5=0;
    //bn*1000 for FIR
    float b1=.0088;
    float b2=.0479;
    float b3=.1640;
    float b4=.2793;
    float b5=.2793;
    float b6=.1640;
    float b7=.0479;
    float b8=.0088;

    int FIRbuf1=0;
    int FIRbuf2=0;
    int FIRbuf3=0;
    int FIRbuf4=0;
    int FIRbuf5=0;
    int FIRbuf6=0;
    int FIRbuf7=0;
    int FIRbuf8=0;


    //sprintf(message,"Hello!");
    //use_display(20,20,message);
    //display_draw();
    /* Check if device is configured.  See if it is configured with correct
     * configuration value  */

    switch(appData.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData.usbDevHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData.usbDevHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData.usbDevHandle, APP_USBDeviceEventHandler, 0);

                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            if(appData.deviceConfigured == true)
            {
                /* Device is ready to run the main task */
                appData.hidDataReceived = false;
                appData.hidDataTransmitted = true;
                appData.state = APP_STATE_MAIN_TASK;

                /* Place a new read request. */
                USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                        &appData.rxTransferHandle, appData.receiveDataBuffer, 64);
            }
            break;

        case APP_STATE_MAIN_TASK:

            if(!appData.deviceConfigured)
            {
                /* Device is not configured */
                appData.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else if( appData.hidDataReceived )
            {
                /* Look at the data the host sent, to see what
                 * kind of application specific command it sent. */

                switch(appData.receiveDataBuffer[0])
                {
                    case 0x80:
                        /* Toggle on board LED1 to LED2. */
                        BSP_LEDToggle( APP_USB_LED_1 );
                        BSP_LEDToggle( APP_USB_LED_2 );
                        
                        for (i=0; i<8; i++){
                            inputfrompc[i] = appData.receiveDataBuffer[i+1];
                        }
                        use_display(20,20,inputfrompc);
                        display_draw();
                        appData.hidDataReceived = false;

                        /* Place a new read request. */
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );

                        break;

                    case 0x81:

                        if(appData.hidDataTransmitted)
                        {
                            /* Echo back to the host PC the command we are fulfilling in
                             * the first byte.  In this case, the Get Push-button State
                             * command. */

                            appData.transmitDataBuffer[0] = 0x81;

                            acc_read_register(OUT_X_L_A, (unsigned char *) accels, 6);
                            sprintf(message,"Z: %d",accels[0]);
                            use_display(10,10,message);
                            display_draw();
                            if(_CP0_GET_COUNT()>800000){
                                appData.transmitDataBuffer[1] = 1;
                                appData.transmitDataBuffer[2] = accels[0]>>8;
                                appData.transmitDataBuffer[3] = accels[0]&0xFF;

                                //use MAF buffer values to calculate accelsMAF
                                accelsMAF = ((buf1+buf2+buf3+buf4+buf5+accels[0])/6);

                                //change MAF buffer values
                                buf5=buf4;
                                buf4=buf3;
                                buf3=buf2;
                                buf2=buf1;
                                buf1=accels[0];



                                //change FIR buffer values
                                FIRbuf8=FIRbuf7;
                                FIRbuf7=FIRbuf6;
                                FIRbuf6=FIRbuf5;
                                FIRbuf5=FIRbuf4;
                                FIRbuf4=FIRbuf3;
                                FIRbuf3=FIRbuf2;
                                FIRbuf2=FIRbuf1;
                                FIRbuf1=accels[0];
                                //FIR Filtering calculations
                                accelsFIR = (b1*FIRbuf1)+(b2*FIRbuf2)+(b3*FIRbuf3)+(b4*FIRbuf4)+(b5*FIRbuf5)+(b6*FIRbuf6)+(b7*FIRbuf7)+(b8*FIRbuf8);

                                appData.transmitDataBuffer[4] = accelsMAF>>8;
                                appData.transmitDataBuffer[5] = accelsMAF&0xFF;
                                appData.transmitDataBuffer[6] = accelsFIR>>8;
                                appData.transmitDataBuffer[7] = accelsFIR&0xFF;
                                _CP0_SET_COUNT(0);

                            }
                            else{appData.transmitDataBuffer[1]=0;}


                            

                            appData.hidDataTransmitted = false;

                            /* Prepare the USB module to send the data packet to the host */
                            USB_DEVICE_HID_ReportSend (USB_DEVICE_HID_INDEX_0,
                                    &appData.txTransferHandle, appData.transmitDataBuffer, 64 );

                            appData.hidDataReceived = false;

                            /* Place a new read request. */
                            USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                    &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        }
                        break;

                    default:

                        appData.hidDataReceived = false;

                        /* Place a new read request. */
                        USB_DEVICE_HID_ReportReceive (USB_DEVICE_HID_INDEX_0,
                                &appData.rxTransferHandle, appData.receiveDataBuffer, 64 );
                        break;
                }
            }
        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
 
int use_display(int xposition, int yposition, char input[25]) {
    int asciiROW; //row of the table being used
    int maxX = 128;
    int startX = xposition;
    int startY = yposition;
    int yloc = startY;
    int cyclestring = 1;
    int index = 0;
    int adjustASCII = 0x20;
    int i;
    int byte;
    int byteINDEX;
    int val; // a 0 or a 1, this is a bit of a byte from the table of hex numbers
    int var;
    int xloc;


    while (input[index]) {

        var = input[index];
        xloc = startX + (5 * index); //increment X position
        yloc = startY;

        asciiROW = var - adjustASCII; //row of the big ASCII table

        for (i = 0; i < 5; i++) { //sets the column of big ASCII table
            byte = ASCII[asciiROW][i];
            for (byteINDEX = 0; byteINDEX < 8; byteINDEX++) { //sets bit within Hex element of ASCII table
                //set "val" variable equal to value of this bit (0 or 1) from the given hex 0x##
                if ((byte >> byteINDEX) & 1) { //compare bitwise to establish "val" as 1 or 0
                    val = 1;
                } else {
                    val = 0;
                }
                display_pixel_set(yloc + byteINDEX, xloc + i, val);

            }
        }

        index++; //look at the next element of the string being printed
    }
    return (0);
}
/*******************************************************************************
 End of File
 */

