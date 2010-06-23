

// Standard Input/Output functions
//#include <iostream>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>                               // for in-/output
#include <string.h>                              // strcat
#include <fcntl.h>                               // for 'O_RDONLY' deklaration
#include <termios.h>                             // for serial
// For p_thread :
#include <pthread.h>
// For I2C : 
#include <linux/i2c-dev.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "gumadc.h"
//#include "../adc/gumadc.h"

#ifdef __cplusplus
}
#endif

#ifndef TEMERAIRE_H
#define TEMERAIRE_H

#define NEW_INPUT_PROTOCOL
#define FOOT_SENSORS

#define US_DEVICE 0x70

#define SSCDEVICE "/dev/ttyS0"
#define MODEMDEVICE "/dev/rfcomm0"
#define BAUDRATE B115200

#define False 0
#define True  1 
#define FALSE 0
#define TRUE  1


#define BUTTON_DOWN 0
#define BUTTON_UP   1

// SERVO OFFSET
#define SERVO_OFFSET0   23
#define SERVO_OFFSET1   64
#define SERVO_OFFSET2   155

#define SERVO_OFFSET4   107
#define SERVO_OFFSET5   19
//#define SERVO_OFFSET6   40
#define SERVO_OFFSET6   92

#define SERVO_OFFSET8   -24
//#define SERVO_OFFSET9   0
#define SERVO_OFFSET9   55
//#define SERVO_OFFSET10  0
#define SERVO_OFFSET10  -131


#define SERVO_OFFSET16  -40
#define SERVO_OFFSET17  -32
#define SERVO_OFFSET18  108

#define SERVO_OFFSET20  27
#define SERVO_OFFSET21  69
#define SERVO_OFFSET22  123

#define SERVO_OFFSET24  101
#define SERVO_OFFSET25  -27
//#define SERVO_OFFSET26  0
#define SERVO_OFFSET26  120


/*
// SERVO OFFSET
#define SERVO_OFFSET0   23
#define SERVO_OFFSET1   0
#define SERVO_OFFSET2   20

#define SERVO_OFFSET4   107
#define SERVO_OFFSET5   -19
//#define SERVO_OFFSET6   40
#define SERVO_OFFSET6   48

#define SERVO_OFFSET8   -24
//#define SERVO_OFFSET9   0
#define SERVO_OFFSET9   1
//#define SERVO_OFFSET10  0
#define SERVO_OFFSET10  12
    

#define SERVO_OFFSET16  -40
#define SERVO_OFFSET17  -50
#define SERVO_OFFSET18  40

#define SERVO_OFFSET20  27
#define SERVO_OFFSET21  100
#define SERVO_OFFSET22  40

#define SERVO_OFFSET24  101
#define SERVO_OFFSET25  48
//#define SERVO_OFFSET26  0
#define SERVO_OFFSET26  -27
*/

//[MIN/MAX ANGLES]
#define RRCoxa_MIN -26      //Mechanical limits of the Right Rear Leg
#define RRCoxa_MAX 74
#define RRFemur_MIN -101
#define RRFemur_MAX 95
#define RRTibia_MIN -106
#define RRTibia_MAX 77

#define RMCoxa_MIN -53      //Mechanical limits of the Right Middle Leg
#define RMCoxa_MAX 53
#define RMFemur_MIN -101
#define RMFemur_MAX 95
#define RMTibia_MIN -106
#define RMTibia_MAX 77

#define RFCoxa_MIN -58      //Mechanical limits of the Right Front Leg
#define RFCoxa_MAX 74
#define RFFemur_MIN -101
#define RFFemur_MAX 95
#define RFTibia_MIN -106
#define RFTibia_MAX 77

#define LRCoxa_MIN -74      //Mechanical limits of the Left Rear Leg
#define LRCoxa_MAX 26
#define LRFemur_MIN -95
#define LRFemur_MAX 101
#define LRTibia_MIN -77
#define LRTibia_MAX 106

#define LMCoxa_MIN -53      //Mechanical limits of the Left Middle Leg
#define LMCoxa_MAX 53
#define LMFemur_MIN -95
#define LMFemur_MAX 101
#define LMTibia_MIN -77
#define LMTibia_MAX 106

#define LFCoxa_MIN -74      //Mechanical limits of the Left Front Leg
#define LFCoxa_MAX 58
#define LFFemur_MIN -95
#define LFFemur_MAX 101
#define LFTibia_MIN -77
#define LFTibia_MAX 106
//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define CoxaLength 29      //Length of the Coxa [mm]
#define FemurLength 76      //Length of the Femur [mm]
//#define TibiaLength 106     //Lenght of the Tibia [mm]
#define TibiaLength 120     //NEW Lenght of the Tibia [mm]

#define FemurLength2 197      //Length of the Femur [mm]
#define TibiaLength2 158     //Lenght of the Tibia [mm] 242

#define CoxaAngle 60      //Default Coxa setup angle

#define RFOffsetX -43      //Distance X from center of the body to the Right Front coxa
#define RFOffsetZ -82      //Distance Z from center of the body to the Right Front coxa
#define RMOffsetX -63      //Distance X from center of the body to the Right Middle coxa
#define RMOffsetZ 0        //Distance Z from center of the body to the Right Middle coxa
#define RROffsetX -43      //Distance X from center of the body to the Right Rear coxa
#define RROffsetZ 82       //Distance Z from center of the body to the Right Rear coxa

#define LFOffsetX 43      //Distance X from center of the body to the Left Front coxa
#define LFOffsetZ -82     //Distance Z from center of the body to the Left Front coxa
#define LMOffsetX 63      //Distance X from center of the body to the Left Middle coxa
#define LMOffsetZ 0       //Distance Z from center of the body to the Left Middle coxa
#define LROffsetX 43      //Distance X from center of the body to the Left Rear coxa
#define LROffsetZ 82      //Distance Z from center of the body to the Left Rear coxa
//--------------------------------------------------------------------
//[REMOTE]
#define TravelDeadZone 4   //The deadzone for the analog input from the remote

#define LIMIT_HEIGHT 10


#endif
