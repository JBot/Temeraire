
#include "temeraire.h"

#ifndef TEMERAIRE_VAR_H
#define TEMERAIRE_VAR_H

signed int ARMCoxaAngle = 0;   
signed int ARMFemurAngle = 0;
signed int ARMTibiaAngle = 0;

signed int X, Y, Z, Xbase, Ybase, Zbase;
#ifdef WRITINGBOT
signed int armtab[11][3];
char indexarmtab;
#endif


//====================================================================
//[ANGLES]
signed int RFCoxaAngle = 0;   //Actual Angle of the Right Front Leg
signed int RFFemurAngle = 0;
signed int RFTibiaAngle = 0;

signed int RMCoxaAngle = 0;   //Actual Angle of the Right Middle Leg
signed int RMFemurAngle = 0;
signed int RMTibiaAngle = 0;

signed int RRCoxaAngle = 0;   //Actual Angle of the Right Rear Leg
signed int RRFemurAngle = 0;
signed int RRTibiaAngle = 0;

signed int LFCoxaAngle = 0;   //Actual Angle of the Left Front Leg
signed int LFFemurAngle = 0;
signed int LFTibiaAngle = 0;

signed int LMCoxaAngle = 0;   //Actual Angle of the Left Middle Leg
signed int LMFemurAngle = 0;
signed int LMTibiaAngle = 0;

signed int LRCoxaAngle = 0;   //Actual Angle of the Left Rear Leg
signed int LRFemurAngle = 0;
signed int LRTibiaAngle = 0;

signed int horizontal_turret;
signed int vertical_turret;
//--------------------------------------------------------------------
//[POSITIONS]
signed int RFPosX = 0;      //Actual Position of the Right Front Leg
signed int RFPosY = 0;
signed int RFPosZ = 0;

signed int RMPosX = 0;      //Actual Position of the Right Middle Leg
signed int RMPosY = 0;
signed int RMPosZ = 0;

signed int RRPosX = 0;      //Actual Position of the Right Rear Leg
signed int RRPosY = 0;
signed int RRPosZ = 0;

signed int LFPosX = 0;      //Actual Position of the Left Front Leg
signed int LFPosY = 0;
signed int LFPosZ = 0;

signed int LMPosX = 0;      //Actual Position of the Left Middle Leg
signed int LMPosY = 0;
signed int LMPosZ = 0;

signed int LRPosX = 0;      //Actual Position of the Left Rear Leg
signed int LRPosY = 0;
signed int LRPosZ = 0;


//--------------------------------------------------------------------
//[VARIABLES]
char Index;      //Index used for freeing the servos
char SSCDone;    //Char to check if SSC is done

//GetSinCos
//float AngleDeg;      //Input Angle in degrees
float ABSAngleDeg;      //Absolute value of the Angle in Degrees
float AngleRad;     //Angle in Radian
float SinA;      //Output Sinus of the given Angle
float CosA;      //Output Cosinus of the given Angle

//GetBoogTan (Atan)
//signed int BoogTanX;      //Input X
//signed int BoogTanY;      //Input Y
float BoogTan;      //Output BOOGTAN2(X/Y)

//Body position
signed int BodyPosX = 0;      //Global Input for the position of the body
signed int BodyPosY = 0;
signed int BodyPosZ = 0;

signed int BodyPosXint = 0;
signed int BodyPosZint = 0; 
signed int BodyPosYint = 0;


//Body Inverse Kinematics
signed char BodyRotX = 0;          //Global Input pitch of the body
signed char BodyRotY = 0;          //Global Input rotation of the body
signed char BodyRotZ = 0;          //Global Input roll of the body
//signed int PosX;               //Input position of the feet X
//signed int PosZ;               //Input position of the feet Z
//signed int PosY;               //Input position of the feet Y
//signed char RotationY;         //Input for rotation of a single feet for the gait
//signed char BodyOffsetX;       //Input Offset betweeen the body and Coxa X
//signed char BodyOffsetZ;       //Input Offset betweeen the body and Coxa Z
float SinB;             //Sin buffer for BodyRotX calculations
float CosB;             //Cos buffer for BodyRotX calculations
float SinG;             //Sin buffer for BodyRotZ calculations
float CosG;             //Cos buffer for BodyRotZ calculations
signed int TotalX;             //Total X distance between the center of the body and the feet
signed int TotalZ;             //Total Z distance between the center of the body and the feet
float DistCenterBodyFeet; //Total distance between the center of the body and the feet
float AngleCenterBodyFeetX; //Angle between the center of the body and the feet
signed int BodyIKPosX;         //Output Position X of feet with Rotation
signed int BodyIKPosY;         //Output Position Y of feet with Rotation
signed int BodyIKPosZ;         //Output Position Z of feet with Rotation


//Leg Inverse Kinematics
//signed int IKFeetPosX;         //Input position of the Feet X
//signed int IKFeetPosY;         //Input position of the Feet Y
//signed int IKFeetPosZ;         //Input Position of the Feet Z
signed int IKFeetPosXZ;        //Length between the coxa and feet
float IKSW;             //Length between shoulder and wrist
float IKA1;             //Angle between SW line and the ground in rad
float IKA2;             //?
char IKSolution;         //Output true if the solution is possible
char IKSolutionWarning;      //Output true if the solution is NEARLY possible
char IKSolutionError;      //Output true if the solution is NOT possible
signed int IKFemurAngle;       //Output Angle of Femur in degrees
signed int IKTibiaAngle;       //Output Angle of Tibia in degrees
signed int IKCoxaAngle;        //Output Angle of Coxa in degrees

char ResetInitPos;
char Mode;    //ch5 Mode switch + twostate switch H = 6 modes  
char TestLeg;

//--------------------------------------------------------------------
//[GLOABAL]
char HexOn;        //Switch to turn on Phoenix
char TurnOff;        //Mark to turn off Phoenix
//--------------------------------------------------------------------
//[Balance]
char BalanceMode;
signed int TravelHeightY = 0;
signed int TotalTransX = 0;
signed int TotalTransZ = 0;
signed int TotalTransY = 0;
signed int TotalYBal = 0;
signed int TotalXBal = 0;
signed int TotalZBal = 0;
signed int TotalY = 0;       //Total Y distance between the center of the body and the feet

//[gait]
char GaitType;   //Gait type
int NomGaitSpeed;   //Nominal speed of the gait
int ActualGaitSpeed;

signed int LegLiftHeight;   //Current Travel height
signed int TravelLengthX;   //Current Travel length X
signed int TravelLengthZ;   //Current Travel length Z
signed int TravelRotationY;   //Current Travel Rotation Y

signed int TLDivFactor;   //Number of steps that a leg is on the floor while walking
char NrLiftedPos;      //Number of positions that a single leg is lifted (1-3)
char HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height   

char GaitInMotion;      //Temp to check if the gait is in motion
char StepsInGait;   //Number of steps in gait
char LastLeg;      //TRUE when the current leg is the last leg of the sequence
char GaitStep;   //Actual Gait step

signed int RFGaitLegNr = 0;   //Init position of the leg
signed int RMGaitLegNr = 0;   //Init position of the leg
signed int RRGaitLegNr = 0;   //Init position of the leg
signed int LFGaitLegNr = 0;   //Init position of the leg
signed int LMGaitLegNr = 0;   //Init position of the leg
signed int LRGaitLegNr = 0;   //Init position of the leg

//char GaitLegNr;   //Input Number of the leg
signed int TravelMulti;   //Multiplier for the length of the step

signed int RFGaitPosX = 0;   //Relative position corresponding to the Gait
signed int RFGaitPosY = 0;
signed int RFGaitPosZ = 0;
signed int RFGaitRotY = 0;   //Relative rotation corresponding to the Gait

signed int RMGaitPosX = 0;
signed int RMGaitPosY = 0;
signed int RMGaitPosZ = 0;
signed int RMGaitRotY = 0;

signed int RRGaitPosX = 0;
signed int RRGaitPosY = 0;
signed int RRGaitPosZ = 0;
signed int RRGaitRotY = 0;

signed int LFGaitPosX = 0;
signed int LFGaitPosY = 0;
signed int LFGaitPosZ = 0;
signed int LFGaitRotY = 0;

signed int LMGaitPosX = 0;
signed int LMGaitPosY = 0;
signed int LMGaitPosZ = 0;
signed int LMGaitRotY = 0;

signed int LRGaitPosX = 0;
signed int LRGaitPosY = 0;
signed int LRGaitPosZ = 0;
signed int LRGaitRotY = 0;

signed int GaitPosX = 0;   //In-/Output Pos X of feet
signed int GaitPosY = 0;   //In-/Output Pos Y of feet
signed int GaitPosZ = 0;   //In-/Output Pos Z of feet
signed int GaitRotY = 0;   //In-/Output Rotation Y of feet

char starting =0;
char sleeping =1;
char display1, display2, display3, display4;
int entier;
signed int headAngle;
signed int down_leg_step = 2;

signed int RotPoint = 0; 
char mylegnumber = 0;

// Serial
int ser_fd_ssc;
struct termios oldtio_ssc, newtio_ssc;
int ser_fd_modem;
struct termios oldtio_modem, newtio_modem;

char wait_command_flag = 1;
char modem_command;
int temp_input = 0;


// ADC
int file_adc;
struct twl4030_madc_user_parms *par;

// I2C
int file_i2c;
int us_sensor_distance = 255;
int us_sensor_light = 255;

// Thread
pthread_t p_thread[10];

// Mandibles
int Mandible = 1500;

// GPIO
int file_gpio146;
char gpio146_input[10];
int file_gpio147;
char gpio147_input[10];
int file_gpio114;
char gpio114_input[10];
int file_gpio186;
char gpio186_input[10];
int file_gpio144;
char gpio144_input[10];
int file_gpio145;
char gpio145_input[10];

// Legs sensors
char leg_on_floor = 1;
#define DOWN_SENSOR_SPEED 20

#endif
