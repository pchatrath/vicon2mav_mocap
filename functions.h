/* functions.h
 * author: Julian L. Nicklas, julian.nicklas (at) posteo (.) de
 */

#include <sys/time.h>   // for gettimeofteday();
#include <time.h>       // for nanoslepp(...);
#include <math.h>
#include <cstdlib>

#include "quaternions.h"

//##################### constants ########################

#define VICON_SERVER_ADDRESS "192.168.0.12:801"        // ip of the windows PC
#define UAV_NAME "AR2_201"            // the subject name of this UAV, this application only listens for data for this subject name
#define MY_NET_ADDRESS INADDR_ANY
#define TCP2SERIAL_ADRESS "127.0.0.1"
#define TCP2SERIAL_PORT 5763
#define FREQUENCY 30 // maximum frequency of the main loop in Hz
                // if the code is slower this frequency will not be archieved

#define PI 3.14159265
//#define ROOM_ALIGNMENT 0 // Angle between North- and x-axis of the Vicon coordinate system (positions send via MavLink should be in NED)
        // Angle is measured in radians positive around the z-up-Axis from Vicon x-axis to North-axis
#define ROOM_ALIGNMENT 2.474 //room alignment of the Vicon Enviroment at the NPS


//################# function declarations ###################

void coordinateTrans_ViconToNED(float *viconPosPointer, float *rotation1_quat, float *rotation2_quat);

void millisleep(unsigned int seconds);
unsigned long long int getMSecondoftheday();
unsigned long long int getMicroSecondoftheday();


/**
 * Converts Position and Attitude Information of the Vicon Coordinate System to the NED Coordinate System
 *
 * *viconPosPointer Vicon Position and Attutide Data: (x, y, z, x_quat, y_quat, z_quat, w_quat)
 * *rotation1_quat first rotation: (w_quat, x_quat, y_quat, z_quat)
 * *rotation2_quat second rotation: (w_quat, x_quat, y_quat, z_quat)
 *
 * rotation_quat are only used to transform the attitude !!
 * the position transformation is defined inside this function !!
 */
void coordinateTrans_ViconToNED(float *viconPosPointer, float *rotation1_quat, float *rotation2_quat)
{
    float NED_Position[3];
    float NED_Attitude[4];
    float viconPosC[7];

    for(int i = 0; i < 7; ++i)
        viconPosC[i] = *((float*)(viconPosPointer + i));

    // --- Position ---
    // x_NED = (cos()*x_vicon + sin()*y_vicon)/1000
    NED_Position[0] = (cos(ROOM_ALIGNMENT)*viconPosC[0] + sin(ROOM_ALIGNMENT)*viconPosC[1]) / 1000;
    // y_NED = (sin()*x_vicon - cos()*y_vicon)/1000
    NED_Position[1] = (sin(ROOM_ALIGNMENT)*viconPosC[0] - cos(ROOM_ALIGNMENT)*viconPosC[1]) / 1000;
    // z_NED = -z_vicon/1000
    NED_Position[2] = - viconPosC[2] / 1000;

    // --- Quaternions ---
    // vicon: (x, y, z, z_quat, x_quat, y_quat, w_quat) -> common: (w_quat, x_quat, y_quat, z_quat)
    NED_Attitude[0] = viconPosC[6];
    NED_Attitude[1] = viconPosC[3];
    NED_Attitude[2] = viconPosC[4];
    NED_Attitude[3] = viconPosC[5];
    // the Attitude Quaternion is first rotated according to rotation1 and after that according
    // to rotation2
    quat_rotate(NED_Attitude, rotation1_quat);
    quat_rotate(NED_Attitude, rotation2_quat);


    // Change the order of the variables from Vicon -> att_pos_mocap
    // (x, y, z, w_quat, x_quat, y_quat, z_quat) -> (w_quat, x_quat, y_quat, z_quat, x, y, z)
    *(viconPosPointer)      = NED_Attitude[0];
    *(viconPosPointer + 1)  = NED_Attitude[1];
    *(viconPosPointer + 2)  = NED_Attitude[2];
    *(viconPosPointer + 3)  = NED_Attitude[3];
    *(viconPosPointer + 4)  = NED_Position[0];
    *(viconPosPointer + 5)  = NED_Position[1];
    *(viconPosPointer + 6)  = NED_Position[2];
}

void millisleep(unsigned int milliseconds) // sleep for a certain amount of Milliseconds
{
    struct timespec WaitFor = {0,0};

    div_t divresult = div(milliseconds,1000);
    WaitFor.tv_sec = divresult.quot;
    WaitFor.tv_nsec = divresult.rem*1000000;

    nanosleep(&WaitFor, (struct timespec *)NULL);
}

unsigned long long int getMSecondoftheday() // get the UNIX time in Milliseconds
{
    struct timeval timeNow;
    gettimeofday(&timeNow, NULL);
    return timeNow.tv_sec*1000 + timeNow.tv_usec/1000;
}

unsigned long long int getMicroSecondoftheday() // get the UNIX time in Microseconds
{
    struct timeval timeNow;
    gettimeofday(&timeNow, NULL);
    return timeNow.tv_sec*1000000 + timeNow.tv_usec;
}
