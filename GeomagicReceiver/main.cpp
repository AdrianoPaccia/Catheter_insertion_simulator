#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>
#include <cmath>
#include <conio.h> 

#include <HD/hd.h>
#include <HDU/hduError.h>


#include<winsock2.h>
#pragma comment(lib,"ws2_32.lib") //Winsock Library

#include <sstream>
#include <string>

#include <chrono>




/*******************************************************************************
 Globals, declarations etc 
*******************************************************************************/

/* Current button state. */
int btn[ 3 ] = { 0, 0, 0 };        
/* Current mouse position. */
int mouse_x, mouse_y;            


HHD ghHD = HD_INVALID_HANDLE;
HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;


/******************************************************************************
 Haptic device record 
******************************************************************************/
struct DeviceDisplayState
{
    HHD m_hHD;
    double EEPosition[3];
    double EEVelocity[3];
    double jointVariables[3];
    double gimbalAngularVelocity[3];
    double gimbalAngle[3];
    double currentEEForce[3];
    HDint currentButtons;
    HDint lastButtons; 
};

DeviceDisplayState geomagicVariables; //global variable with updated informations about the haptic device

/*******************************************************************************
 UDP Server Declarations, globals etc
*******************************************************************************/

#define SERVER "127.0.0.1"	//ip address of udp server
#define BUFLEN 1024	//Max length of buffer
#define PORT 20001	//The port on which to listen for incoming data
#define UDP_LOOP_TIME 150
#define UDP_TIMEOUT_MSEC 10
auto last_exec_time = std::chrono::high_resolution_clock::now();
auto current_time = std::chrono::high_resolution_clock::now();

char bufferUDP[BUFLEN];
SOCKET s = -1; //variable containing udp server socket
WSADATA wsa; //variable containing winsock data
struct sockaddr_in si_other; // more server data
int slen = sizeof(si_other); //server data length
DWORD timeout = UDP_TIMEOUT_MSEC;



/******************************************************************************
 Query haptic device state: position, force, etc.
******************************************************************************/
void updateGeomagicState()
{


    HHD hHD = hdGetCurrentDevice();

    hdBeginFrame(hHD);
    hdGetDoublev(HD_CURRENT_POSITION, geomagicVariables.EEPosition);
    hdGetDoublev(HD_CURRENT_VELOCITY, geomagicVariables.EEVelocity);

    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, geomagicVariables.jointVariables);

    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, geomagicVariables.gimbalAngle);
    hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, geomagicVariables.gimbalAngularVelocity);

    hdGetIntegerv(HD_CURRENT_BUTTONS, &geomagicVariables.currentButtons);
    hdGetIntegerv(HD_LAST_BUTTONS, &geomagicVariables.lastButtons);

    hdSetDoublev(HD_CURRENT_FORCE, geomagicVariables.currentEEForce);

    hdEndFrame(hHD);

    //hduVector3Dd position;

}

/******************************************************************************
 Print haptic device state: position, force, etc.
******************************************************************************/
void printGeomagicState()
{
    /*
    printf("----------------------\n");
    printf("eePos: %f,%f,%f\n", geomagicVariables.EEPosition[0], geomagicVariables.EEPosition[1], geomagicVariables.EEPosition[2]);
    printf("eeVel: %f,%f,%f\n", geomagicVariables.EEVelocity[0], geomagicVariables.EEVelocity[1], geomagicVariables.EEVelocity[2]);
    printf("joints: %f,%f,%f\n", geomagicVariables.jointVariables[0], geomagicVariables.jointVariables[1], geomagicVariables.jointVariables[2]);
    printf("gimbalAngles: %f,%f,%f\n", geomagicVariables.gimbalAngle[0], geomagicVariables.gimbalAngle[1], geomagicVariables.gimbalAngle[2]);
    printf("gimbalAngVel: %f,%f,%f\n", geomagicVariables.gimbalAngularVelocity[0], geomagicVariables.gimbalAngularVelocity[1], geomagicVariables.gimbalAngularVelocity[2]);
    printf("newButtons: %d, lastButtons:%d\n", geomagicVariables.currentButtons, geomagicVariables.lastButtons);
    printf("----------------------\n");
    */
}

/*******************************************************************************
 Connect to the udp server
*******************************************************************************/
int initUDP() {

    //Initialise winsock
    printf("\nInitialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("Failed. Error Code : %d", WSAGetLastError());
        return -1;
    }
    printf("Initialised.\n");

    //create socket
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == SOCKET_ERROR)
    {
        printf("socket() failed with error code : %d\n", WSAGetLastError());
        return -1;
    }
    printf("Socket created.\n");

    //setup address structure
    memset((char*)&si_other, 0, sizeof(si_other));
    si_other.sin_family = AF_INET;
    si_other.sin_port = htons(PORT);
    si_other.sin_addr.S_un.S_addr = inet_addr(SERVER);

    //setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)); //setting timeout
    return 0;
}

/*******************************************************************************
 Send data to the udp server
*******************************************************************************/
int sendDataUdp() {
    //clear the buffer by filling null, it might have previously received data
    memset(bufferUDP, '\0', BUFLEN);

    sprintf(bufferUDP, "X%.2f,Y%.2f,Z%.2f,Q%.2f,W%.2f,E%.2f,B%d", geomagicVariables.EEPosition[0], geomagicVariables.EEPosition[1], geomagicVariables.EEPosition[2], geomagicVariables.gimbalAngle[0], geomagicVariables.gimbalAngle[1], geomagicVariables.gimbalAngle[2], geomagicVariables.currentButtons);
    //printf("Sending message : %s\n", bufferUDP);
    //send the message
    if (sendto(s, bufferUDP, strlen(bufferUDP), 0, (struct sockaddr*)&si_other, slen) == SOCKET_ERROR)
    {
        printf("sendto() failed with error code : %d\n", WSAGetLastError());
        return -1;
    }
    return 0;
}

/*******************************************************************************
 Receive data from udp server
*******************************************************************************/
int recvDataUdp() {
    //receive a reply and print it
    //clear the buffer by filling null, it might have previously received data
    memset(bufferUDP, '\0', BUFLEN);
    //try to receive some data, this is a blocking call
    if (recvfrom(s, bufferUDP, BUFLEN, 0, (struct sockaddr*)&si_other, &slen) == SOCKET_ERROR)
    {
        if (WSAGetLastError() != WSAETIMEDOUT) {
            printf("recvfrom() failed with error code : %d\n", WSAGetLastError());
            return -1;
        }
        else {
            printf("simple timeout\n");
            return 0;
        }
    }
    //printf("Received message: %s\n", bufferUDP);
    std::stringstream ss(bufferUDP);
    char x;       ss >> x; //read X
    double d1;    ss >> d1;    // read force on X
    char comma1;   ss >> comma1; //read comma
    char y;       ss >> y; //read Y
    double d2;    ss >> d2;    // read force on Y
    char comma2;   ss >> comma2; //read comma
    char z;       ss >> z; //read Z
    double d3;    ss >> d3;    // read force on Z
    if (comma1 == ',' && comma2 == ',' && x == 'X' && y == 'Y' && z == 'Z') {
        //printf("Message seems to be correct \n");
        //printf("%f,%f,%f\n", d1, d2, d3);
        geomagicVariables.currentEEForce[0] = d1;
        geomagicVariables.currentEEForce[1] = d2;
        geomagicVariables.currentEEForce[2] = d3;

    }
    else {
        printf("Message incorrect: %c %f %c %c %f %c %c %f\n", x, d1, comma1, y, d2, comma2, z, d3);
    }

    return 0;

}

/*******************************************************************************
 Cleanup the udp server
*******************************************************************************/
void exitUDP() {
    closesocket(s);
    WSACleanup();
}
           


/******************************************************************************
 Main function.
******************************************************************************/
int main(int argc, char* argv[])
{
    HDErrorInfo error;
    printf("Starting application\n");

    if (initUDP() == -1) {
        printf("UDP Connection not working!\n");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        return -1;
    }

    //send initial zero state for automaic binding of the socket
    if (sendDataUdp() == -1) {
        printf("UDP SEND not working!\n");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        return -1;
    }

    
        
    // Initialize the device.  This needs to be called before any actions on the
    // device.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        return -1;
    }

    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));
    
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    printf("Press any key to stop\n");

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        return -1;
    }
    

    while (HD_TRUE) {
        do {
            current_time = std::chrono::high_resolution_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_exec_time).count() >= UDP_LOOP_TIME)
            {
                printf("loop!\n");
                updateGeomagicState();
                printGeomagicState();
                
                if (sendDataUdp() == -1) {
                    printf("UDP SEND not working!\n");
                    fprintf(stderr, "\nPress any key to quit.\n");
                    getchar();
                    return -1;
                }
                // receive data as soon as possible
                if (recvDataUdp() == -1) {
                    printf("UDP RECV not working!\n");
                    fprintf(stderr, "\nPress any key to quit.\n");
                    getchar();
                    return -1;
                }
                
                last_exec_time = std::chrono::high_resolution_clock::now();
            }
            
           
            
            
        } while (!_kbhit());      // until a key press detected
        _getch();            // fetch that key press
        printf("Stopping loop\n");
        break;
    }
    
   
    printf("Done\n");
    exitUDP();
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdStopScheduler();
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
    return 0;
}

/******************************************************************************/

