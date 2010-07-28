
#include "temeraire.h"

#ifndef TEMERAIRE_VAR_H
#define TEMERAIRE_VAR_H

extern signed int ARMCoxaAngle;   
extern signed int ARMFemurAngle;
extern signed int ARMTibiaAngle;

extern signed int X, Y, Z, Xbase, Ybase, Zbase;
#ifdef WRITINGBOT
extern signed int armtab[11][3];
extern char indexarmtab;
#endif


//====================================================================
//[ANGLES]
extern signed int RFCoxaAngle;   //Actual Angle of the Right Front Leg
extern signed int RFFemurAngle;
extern signed int RFTibiaAngle;

extern signed int RMCoxaAngle;   //Actual Angle of the Right Middle Leg
extern signed int RMFemurAngle;
extern signed int RMTibiaAngle;

extern signed int RRCoxaAngle;   //Actual Angle of the Right Rear Leg
extern signed int RRFemurAngle;
extern signed int RRTibiaAngle;

extern signed int LFCoxaAngle;   //Actual Angle of the Left Front Leg
extern signed int LFFemurAngle;
extern signed int LFTibiaAngle;

extern signed int LMCoxaAngle;   //Actual Angle of the Left Middle Leg
extern signed int LMFemurAngle;
extern signed int LMTibiaAngle;

extern signed int LRCoxaAngle;   //Actual Angle of the Left Rear Leg
extern signed int LRFemurAngle;
extern signed int LRTibiaAngle;

extern signed int horizontal_turret;
extern signed int vertical_turret;
//--------------------------------------------------------------------
//[POSITIONS]
extern signed int RFPosX;      //Actual Position of the Right Front Leg
extern signed int RFPosY;
extern signed int RFPosZ;

extern signed int RMPosX;      //Actual Position of the Right Middle Leg
extern signed int RMPosY;
extern signed int RMPosZ;

extern signed int RRPosX;      //Actual Position of the Right Rear Leg
extern signed int RRPosY;
extern signed int RRPosZ;

extern signed int LFPosX;      //Actual Position of the Left Front Leg
extern signed int LFPosY;
extern signed int LFPosZ;

extern signed int LMPosX;      //Actual Position of the Left Middle Leg
extern signed int LMPosY;
extern signed int LMPosZ;

extern signed int LRPosX;      //Actual Position of the Left Rear Leg
extern signed int LRPosY;
extern signed int LRPosZ;


//--------------------------------------------------------------------
//[VARIABLES]
extern char Index;      //Index used for freeing the servos
extern char SSCDone;    //Char to check if SSC is done

//GetSinCos
//float AngleDeg;      //Input Angle in degrees
extern float ABSAngleDeg;      //Absolute value of the Angle in Degrees
extern float AngleRad;     //Angle in Radian
extern float SinA;      //Output Sinus of the given Angle
extern float CosA;      //Output Cosinus of the given Angle

//GetBoogTan (Atan)
//signed int BoogTanX;      //Input X
//signed int BoogTanY;      //Input Y
extern float BoogTan;      //Output BOOGTAN2(X/Y)

//Body position
extern signed int BodyPosX;      //Global Input for the position of the body
extern signed int BodyPosY;
extern signed int BodyPosZ;

extern signed int BodyPosXint;
extern signed int BodyPosZint; 
extern signed int BodyPosYint;


//Body Inverse Kinematics
extern signed char BodyRotX;          //Global Input pitch of the body
extern signed char BodyRotY;          //Global Input rotation of the body
extern signed char BodyRotZ;          //Global Input roll of the body
//signed int PosX;               //Input position of the feet X
//signed int PosZ;               //Input position of the feet Z
//signed int PosY;               //Input position of the feet Y
//signed char RotationY;         //Input for rotation of a single feet for the gait
//signed char BodyOffsetX;       //Input Offset betweeen the body and Coxa X
//signed char BodyOffsetZ;       //Input Offset betweeen the body and Coxa Z
extern float SinB;             //Sin buffer for BodyRotX calculations
extern float CosB;             //Cos buffer for BodyRotX calculations
extern float SinG;             //Sin buffer for BodyRotZ calculations
extern float CosG;             //Cos buffer for BodyRotZ calculations
extern signed int TotalX;             //Total X distance between the center of the body and the feet
extern signed int TotalZ;             //Total Z distance between the center of the body and the feet
extern float DistCenterBodyFeet; //Total distance between the center of the body and the feet
extern float AngleCenterBodyFeetX; //Angle between the center of the body and the feet
extern signed int BodyIKPosX;         //Output Position X of feet with Rotation
extern signed int BodyIKPosY;         //Output Position Y of feet with Rotation
extern signed int BodyIKPosZ;         //Output Position Z of feet with Rotation


//Leg Inverse Kinematics
//signed int IKFeetPosX;         //Input position of the Feet X
//signed int IKFeetPosY;         //Input position of the Feet Y
//signed int IKFeetPosZ;         //Input Position of the Feet Z
extern signed int IKFeetPosXZ;        //Length between the coxa and feet
extern float IKSW;             //Length between shoulder and wrist
extern float IKA1;             //Angle between SW line and the ground in rad
extern float IKA2;             //?
extern char IKSolution;         //Output true if the solution is possible
extern char IKSolutionWarning;      //Output true if the solution is NEARLY possible
extern char IKSolutionError;      //Output true if the solution is NOT possible
extern signed int IKFemurAngle;       //Output Angle of Femur in degrees
extern signed int IKTibiaAngle;       //Output Angle of Tibia in degrees
extern signed int IKCoxaAngle;        //Output Angle of Coxa in degrees

extern char ResetInitPos;
extern char Mode;    //ch5 Mode switch + twostate switch H = 6 modes  
extern char TestLeg;

//--------------------------------------------------------------------
//[GLOABAL]
extern char HexOn;        //Switch to turn on Phoenix
extern char TurnOff;        //Mark to turn off Phoenix
//--------------------------------------------------------------------
//[Balance]
extern char BalanceMode;
extern signed int TravelHeightY;
extern signed int TotalTransX;
extern signed int TotalTransZ;
extern signed int TotalTransY;
extern signed int TotalYBal;
extern signed int TotalXBal;
extern signed int TotalZBal;
extern signed int TotalY;       //Total Y distance between the center of the body and the feet

//[gait]
extern char GaitType;   //Gait type
extern int NomGaitSpeed;   //Nominal speed of the gait
extern int ActualGaitSpeed;

extern signed int LegLiftHeight;   //Current Travel height
extern signed int TravelLengthX;   //Current Travel length X
extern signed int TravelLengthZ;   //Current Travel length Z
extern signed int TravelRotationY;   //Current Travel Rotation Y

extern signed int TLDivFactor;   //Number of steps that a leg is on the floor while walking
extern char NrLiftedPos;      //Number of positions that a single leg is lifted (1-3)
extern char HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height   

extern char GaitInMotion;      //Temp to check if the gait is in motion
extern char StepsInGait;   //Number of steps in gait
extern char LastLeg;      //TRUE when the current leg is the last leg of the sequence
extern char GaitStep;   //Actual Gait step

extern signed int RFGaitLegNr;   //Init position of the leg
extern signed int RMGaitLegNr;   //Init position of the leg
extern signed int RRGaitLegNr;   //Init position of the leg
extern signed int LFGaitLegNr;   //Init position of the leg
extern signed int LMGaitLegNr;   //Init position of the leg
extern signed int LRGaitLegNr;   //Init position of the leg

//char GaitLegNr;   //Input Number of the leg
extern signed int TravelMulti;   //Multiplier for the length of the step

extern signed int RFGaitPosX;   //Relative position corresponding to the Gait
extern signed int RFGaitPosY;
extern signed int RFGaitPosZ;
extern signed int RFGaitRotY;   //Relative rotation corresponding to the Gait

extern signed int RMGaitPosX;
extern signed int RMGaitPosY;
extern signed int RMGaitPosZ;
extern signed int RMGaitRotY;

extern signed int RRGaitPosX;
extern signed int RRGaitPosY;
extern signed int RRGaitPosZ;
extern signed int RRGaitRotY;

extern signed int LFGaitPosX;
extern signed int LFGaitPosY;
extern signed int LFGaitPosZ;
extern signed int LFGaitRotY;

extern signed int LMGaitPosX;
extern signed int LMGaitPosY;
extern signed int LMGaitPosZ;
extern signed int LMGaitRotY;

extern signed int LRGaitPosX;
extern signed int LRGaitPosY;
extern signed int LRGaitPosZ;
extern signed int LRGaitRotY;

extern signed int GaitPosX;   //In-/Output Pos X of feet
extern signed int GaitPosY;   //In-/Output Pos Y of feet
extern signed int GaitPosZ;   //In-/Output Pos Z of feet
extern signed int GaitRotY;   //In-/Output Rotation Y of feet

extern char starting;
extern char sleeping;
extern char display1, display2, display3, display4;
extern int entier;
extern signed int headAngle;
extern signed int down_leg_step;

extern signed int RotPoint; 
extern char mylegnumber;

// Serial
extern int ser_fd_ssc;
extern struct termios oldtio_ssc, newtio_ssc;
extern int ser_fd_modem;
extern struct termios oldtio_modem, newtio_modem;

extern char wait_command_flag;
extern char modem_command;
extern int temp_input;


// ADC
extern int file_adc;
extern struct twl4030_madc_user_parms *par;

// I2C
extern int file_i2c;
extern int us_sensor_distance_front;
extern int us_sensor_distance_left;
extern int us_sensor_distance_right;
extern int us_sensor_light;

// Thread
extern pthread_t p_thread[10];

// Mandibles
extern int Mandible;

// GPIO
extern int file_gpio146;
extern char gpio146_input[10];
extern int file_gpio147;
extern char gpio147_input[10];
extern int file_gpio114;
extern char gpio114_input[10];
extern int file_gpio186;
extern char gpio186_input[10];
extern int file_gpio144;
extern char gpio144_input[10];
extern int file_gpio145;
extern char gpio145_input[10];

// Legs sensors
extern char leg_sensor_ON;
extern char leg_on_floor;
#define DOWN_SENSOR_SPEED 25

// General state of the robot
struct robot_state {
	int state;

};
extern struct robot_state temeraire_state;

#endif
