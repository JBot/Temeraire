
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

// Declare your global variables here

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
#define SERVO_OFFSET1   0
#define SERVO_OFFSET2   20

#define SERVO_OFFSET4   107
#define SERVO_OFFSET5   -19
#define SERVO_OFFSET6   40

#define SERVO_OFFSET8   -24
#define SERVO_OFFSET9   0
#define SERVO_OFFSET10  0
    

#define SERVO_OFFSET16  -40
#define SERVO_OFFSET17  -50
#define SERVO_OFFSET18  40

#define SERVO_OFFSET20  27
#define SERVO_OFFSET21  100
#define SERVO_OFFSET22  40

#define SERVO_OFFSET24  101
#define SERVO_OFFSET25  48
#define SERVO_OFFSET26  0


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
#define TibiaLength 106     //Lenght of the Tibia [mm]

#define FemurLength2 197      //Length of the Femur [mm]
#define TibiaLength2 158     //Lenght of the Tibia [mm] 242

 signed int ARMCoxaAngle = 0;   
 signed int ARMFemurAngle = 0;
 signed int ARMTibiaAngle = 0;
 
 signed int X, Y, Z, Xbase, Ybase, Zbase;
 #ifdef WRITINGBOT
 signed int armtab[11][3];
 char indexarmtab;
 #endif

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


//====================================================================
//[ANGLES]
signed int RFCoxaAngle;   //Actual Angle of the Right Front Leg
signed int RFFemurAngle;
signed int RFTibiaAngle;

signed int RMCoxaAngle;   //Actual Angle of the Right Middle Leg
signed int RMFemurAngle;
signed int RMTibiaAngle;

signed int RRCoxaAngle;   //Actual Angle of the Right Rear Leg
signed int RRFemurAngle;
signed int RRTibiaAngle;

signed int LFCoxaAngle;   //Actual Angle of the Left Front Leg
signed int LFFemurAngle;
signed int LFTibiaAngle;

signed int LMCoxaAngle;   //Actual Angle of the Left Middle Leg
signed int LMFemurAngle;
signed int LMTibiaAngle;

signed int LRCoxaAngle;   //Actual Angle of the Left Rear Leg
signed int LRFemurAngle;
signed int LRTibiaAngle;

signed int horizontal_turret;
signed int vertical_turret;
//--------------------------------------------------------------------
//[POSITIONS]
signed int RFPosX;      //Actual Position of the Right Front Leg
signed int RFPosY;
signed int RFPosZ;

signed int RMPosX;      //Actual Position of the Right Middle Leg
signed int RMPosY;
signed int RMPosZ;

signed int RRPosX;      //Actual Position of the Right Rear Leg
signed int RRPosY;
signed int RRPosZ;

signed int LFPosX;      //Actual Position of the Left Front Leg
signed int LFPosY;
signed int LFPosZ;

signed int LMPosX;      //Actual Position of the Left Middle Leg
signed int LMPosY;
signed int LMPosZ;

signed int LRPosX;      //Actual Position of the Left Rear Leg
signed int LRPosY;
signed int LRPosZ;


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
signed int BodyPosX;      //Global Input for the position of the body
signed int BodyPosY;
signed int BodyPosZ;

signed int BodyPosXint;
signed int BodyPosZint;  
signed int BodyPosYint;


//Body Inverse Kinematics
signed char BodyRotX;          //Global Input pitch of the body
signed char BodyRotY;          //Global Input rotation of the body
signed char BodyRotZ;          //Global Input roll of the body
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
signed int TravelHeightY;
signed int TotalTransX;
signed int TotalTransZ;
signed int TotalTransY;
signed int TotalYBal;
signed int TotalXBal;
signed int TotalZBal;
signed int TotalY;       //Total Y distance between the center of the body and the feet

//[gait]
char GaitType;   //Gait type
int NomGaitSpeed;   //Nominal speed of the gait

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

signed int RFGaitLegNr;   //Init position of the leg
signed int RMGaitLegNr;   //Init position of the leg
signed int RRGaitLegNr;   //Init position of the leg
signed int LFGaitLegNr;   //Init position of the leg
signed int LMGaitLegNr;   //Init position of the leg
signed int LRGaitLegNr;   //Init position of the leg

//char GaitLegNr;   //Input Number of the leg
signed int TravelMulti;   //Multiplier for the length of the step

signed int RFGaitPosX;   //Relative position corresponding to the Gait
signed int RFGaitPosY;
signed int RFGaitPosZ;
signed int RFGaitRotY;   //Relative rotation corresponding to the Gait

signed int RMGaitPosX;
signed int RMGaitPosY;
signed int RMGaitPosZ;
signed int RMGaitRotY;

signed int RRGaitPosX;
signed int RRGaitPosY;
signed int RRGaitPosZ;
signed int RRGaitRotY;

signed int LFGaitPosX;
signed int LFGaitPosY;
signed int LFGaitPosZ;
signed int LFGaitRotY;

signed int LMGaitPosX;
signed int LMGaitPosY;
signed int LMGaitPosZ;
signed int LMGaitRotY;

signed int LRGaitPosX;
signed int LRGaitPosY;
signed int LRGaitPosZ;
signed int LRGaitRotY;

signed int GaitPosX;   //In-/Output Pos X of feet
signed int GaitPosY;   //In-/Output Pos Y of feet
signed int GaitPosZ;   //In-/Output Pos Z of feet
signed int GaitRotY;   //In-/Output Rotation Y of feet

char starting =0;
char display1, display2, display3, display4;
int entier;
signed int headAngle;

// Serial
int ser_fd_ssc;
struct termios oldtio_ssc, newtio_ssc;
int ser_fd_modem;
struct termios oldtio_modem, newtio_modem;



/*--------------------------------------------------------------------
[GAIT Select] */
void GaitSelect(void) {
  //Gait selector
  if (GaitType == 0) { //Ripple Gait 6 steps
   LRGaitLegNr = 1;
   RFGaitLegNr = 2;   
   LMGaitLegNr = 3;    
   RRGaitLegNr = 4;    
   LFGaitLegNr = 5;    
   RMGaitLegNr = 6;
           
   NrLiftedPos = 1;
   HalfLiftHeigth = FALSE;    
   TLDivFactor = 4;    
   StepsInGait = 6;
   NomGaitSpeed = 150;
  } 
   
  if (GaitType == 1) { //Ripple Gait 12 steps
   LRGaitLegNr = 1;
   RFGaitLegNr = 3;
   LMGaitLegNr = 5;
   RRGaitLegNr = 7;
   LFGaitLegNr = 9;
   RMGaitLegNr = 11;

   NrLiftedPos = 3;
   HalfLiftHeigth = TRUE;
   TLDivFactor = 8;    
   StepsInGait = 12;   
    NomGaitSpeed = 100+60;
  }
   
  if (GaitType == 2) { //Quadripple 9 steps
   LRGaitLegNr = 1;   
   RFGaitLegNr = 2;
   LMGaitLegNr = 4;    
   RRGaitLegNr = 5;
   LFGaitLegNr = 7;
   RMGaitLegNr = 8;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 6;    
   StepsInGait = 9;      
    NomGaitSpeed = 150;
  }   
 
  if (GaitType == 3) { //Tripod 4 steps
   LRGaitLegNr = 3;   
   RFGaitLegNr = 1;
   LMGaitLegNr = 1;
   RRGaitLegNr = 1;
   LFGaitLegNr = 3;
   RMGaitLegNr = 3;
    
   NrLiftedPos = 1;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 2;    
   StepsInGait = 4;      
    NomGaitSpeed = 150;
  }
   
  if (GaitType == 4) { //Tripod 6 steps
   LRGaitLegNr = 4;   
   RFGaitLegNr = 1;
   LMGaitLegNr = 1;
   RRGaitLegNr = 1;
   LFGaitLegNr = 4;
   RMGaitLegNr = 4;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 4;    
   StepsInGait = 6;      
    NomGaitSpeed = 150;
  }
 
  if (GaitType == 5) { //Tripod 8 steps
   LRGaitLegNr = 5;
   RFGaitLegNr = 1;
   LMGaitLegNr = 1;
   RRGaitLegNr = 1;
   LFGaitLegNr = 5;
   RMGaitLegNr = 5;
    
   NrLiftedPos = 3;
   HalfLiftHeigth = TRUE;   
   TLDivFactor = 4;    
   StepsInGait = 8;      
    NomGaitSpeed = 110;
  }
 
  if (GaitType == 6) { //Wave 12 steps
   LRGaitLegNr = 7;
   RFGaitLegNr = 1;
   LMGaitLegNr = 9;
   RRGaitLegNr = 5;
   LFGaitLegNr = 11;
   RMGaitLegNr = 3;
    
   NrLiftedPos = 1;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 10;    
   StepsInGait = 12;      
    NomGaitSpeed = 120;
  }   
 
  if(GaitType == 7) { //Wave 18 steps
   LRGaitLegNr = 10; 
   RFGaitLegNr = 1;
   LMGaitLegNr = 13;
   RRGaitLegNr = 7;
   LFGaitLegNr = 16;
   RMGaitLegNr = 4;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 16;    
   StepsInGait = 18;      
    NomGaitSpeed = 100;
  }
  
  if(GaitType == 8) { // 4 legs
   LRGaitLegNr = 1; 
   RMGaitLegNr = 10;
   LMGaitLegNr = 4;
   RRGaitLegNr = 7;
   
   LFGaitLegNr = 16;
   RFGaitLegNr = 20;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 9;    
   StepsInGait = 12;      
    NomGaitSpeed = 150;
  }
  
  if(GaitType == 10) { //Wave 18 steps
   LRGaitLegNr = 12; 
   RFGaitLegNr = 2;
   LMGaitLegNr = 15;
   RRGaitLegNr = 8;
   LFGaitLegNr = 18;
   RMGaitLegNr = 5;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 18;    
   StepsInGait = 20;      
    NomGaitSpeed = 400;
  }
  
  if(GaitType == 9) { //Wave 14 steps
   LRGaitLegNr = 9; 
   RFGaitLegNr = 2;
   RRGaitLegNr = 5;
   LFGaitLegNr = 12;
   
   RMGaitLegNr = 16;
   LMGaitLegNr = 20;
    
   NrLiftedPos = 2;
   HalfLiftHeigth = FALSE;   
   TLDivFactor = 12;    
   StepsInGait = 14;      
    NomGaitSpeed = 400;
  }
  
return;
}


/**--------------------------------------------------------------------
* PEUT ETRE UTILISER CABS AU LIEU DE ABS
[GAIT]*/
void Gait(char GaitLegNr, signed int GaitPosXX, signed int GaitPosYY, signed int GaitPosZZ, signed int GaitRotYY) {
   if (Mode == 3) {
      if (TestLeg == GaitLegNr) {
         GaitPosX = TravelLengthX;     
            GaitPosY = -TravelHeightY;
            GaitPosZ = TravelLengthZ;
            GaitRotY = 0;
      }
   }
   else {
  //Check IF the Gait is in motion
  if( (abs(TravelLengthX)>TravelDeadZone) || (abs(TravelLengthZ)>TravelDeadZone) || (abs(TravelRotationY)>TravelDeadZone) ) {
         //putchar('J');
         GaitInMotion = 1;
  }
  else {
         GaitInMotion = 0;
  }

  //Leg middle up position
      //Gait in motion                                            Gait NOT in motion, return to home position
  if ((GaitInMotion && (NrLiftedPos==1 || NrLiftedPos==3) && GaitStep==GaitLegNr) || (GaitInMotion==FALSE && GaitStep==GaitLegNr && ((abs(GaitPosXX)>2) || (abs(GaitPosZZ)>2) || (abs(GaitRotYY)>2)))) {   //Up
    GaitPosX = 0;
    GaitPosY = -LegLiftHeight;
    GaitPosZ = 0;
    GaitRotY = 0;
    
  }
  else {

    //Optional Half heigth Rear
    if (((NrLiftedPos==2 && GaitStep==GaitLegNr) || (NrLiftedPos==3 && (GaitStep==(GaitLegNr-1) || GaitStep==GaitLegNr+(StepsInGait-1)))) && GaitInMotion) {
     GaitPosX = -TravelLengthX/2;
      GaitPosY = -LegLiftHeight/((signed int)HalfLiftHeigth+1);
      GaitPosZ = -TravelLengthZ/2;
      GaitRotY = -TravelRotationY/2;
      
      }
     else {
      
     //Optional half heigth front
      if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr+1 || GaitStep==GaitLegNr-(StepsInGait-1)) && GaitInMotion) {
        GaitPosX = TravelLengthX/2;
        GaitPosY = -LegLiftHeight/((signed int)HalfLiftHeigth+1);
        GaitPosZ = TravelLengthZ/2;
        GaitRotY = TravelRotationY/2;
        
        }
      else {     

         //Leg front down position
         if ((GaitStep==GaitLegNr+NrLiftedPos || GaitStep==GaitLegNr-(StepsInGait-NrLiftedPos)) && GaitPosYY<0) {
         GaitPosX = TravelLengthX/2;
          GaitPosY = 0;
          GaitPosZ = TravelLengthZ/2;
          GaitRotY = TravelRotationY/2;
         }
         //Move body forward     
         else {
          GaitPosX = GaitPosXX - (TravelLengthX/TLDivFactor);     
          GaitPosY = 0;
          GaitPosZ = GaitPosZZ - (TravelLengthZ/TLDivFactor);
          GaitRotY = GaitRotYY - (TravelRotationY/TLDivFactor);
        }
      }
    }
  }
  }
  
  
   
  //Advance to the next step
  if (LastLeg) {   //The last leg in this step
    GaitStep = GaitStep+1;
    if (GaitStep>StepsInGait) {
      GaitStep = 1;
    }
  }
       
 return;

}

/**--------------------------------------------------------------------
[GAIT Sequence]*/
void GaitSeq(void) {
  //Calculate Gait sequence
  
  if(GaitType == 9) {
        if(GaitStep < 8){
                BodyPosX = -20;
                if(GaitStep < 4)
                        BodyPosZ = -20;
                else
                        BodyPosZ = 20;        
        }
        else {
                BodyPosX = 20;
                if(GaitStep > 10)
                        BodyPosZ = -20;
                else
                        BodyPosZ = 20;        
        }
  
  }
  
  
  LastLeg = FALSE;
  Gait(LRGaitLegNr, LRGaitPosX, LRGaitPosY, LRGaitPosZ, LRGaitRotY);
 
  LRGaitPosX = GaitPosX;
  LRGaitPosY = GaitPosY;
  LRGaitPosZ = GaitPosZ;
  LRGaitRotY = GaitRotY;   
   
  Gait(RFGaitLegNr, RFGaitPosX, RFGaitPosY, RFGaitPosZ, RFGaitRotY);
  RFGaitPosX = GaitPosX;
  RFGaitPosY = GaitPosY;
  RFGaitPosZ = GaitPosZ;
  RFGaitRotY = GaitRotY;
  
  if(GaitType != 9) { 
  Gait(LMGaitLegNr, LMGaitPosX, LMGaitPosY, LMGaitPosZ, LMGaitRotY);
  LMGaitPosX = GaitPosX;
  LMGaitPosY = GaitPosY;
  LMGaitPosZ = GaitPosZ;
  LMGaitRotY = GaitRotY;   
  }
  
  Gait(RRGaitLegNr, RRGaitPosX, RRGaitPosY, RRGaitPosZ, RRGaitRotY);
  RRGaitPosX = GaitPosX;
  RRGaitPosY = GaitPosY;
  RRGaitPosZ = GaitPosZ;
  RRGaitRotY = GaitRotY;   
  
  if(GaitType == 9) {
  LastLeg = TRUE;
  }
  Gait(LFGaitLegNr, LFGaitPosX, LFGaitPosY, LFGaitPosZ, LFGaitRotY);
  LFGaitPosX = GaitPosX;
  LFGaitPosY = GaitPosY;
  LFGaitPosZ = GaitPosZ;
  LFGaitRotY = GaitRotY;   
  
  if(GaitType != 9) {
  LastLeg = TRUE;
  Gait(RMGaitLegNr, RMGaitPosX, RMGaitPosY, RMGaitPosZ, RMGaitRotY);
  RMGaitPosX = GaitPosX;
  RMGaitPosY = GaitPosY;
  RMGaitPosZ = GaitPosZ;
  RMGaitRotY = GaitRotY;
  }
     
return;
}

/**--------------------------------------------------------------------
[GETSINCOS] Get the sinus and cosinus from the angle +/- multiple circles
AngleDeg    - Input Angle in degrees
SinA        - Output Sinus of AngleDeg
CosA        - Output Cosinus of AngleDeg*/
void GetSinCos (float AngleDeg) {

   //Get the absolute value of AngleDeg

   ABSAngleDeg = fabs(AngleDeg);
 
   
   
   //Shift rotation to a full circle of 360 deg -> AngleDeg // 360
   if (AngleDeg < 0.0) {   //Negative values
      AngleDeg = 360.0-(ABSAngleDeg-((360*((int)(ABSAngleDeg/360.0)))));
   }
   else {            //Positive values
      AngleDeg = ABSAngleDeg-((360*((int)(ABSAngleDeg/360.0))));
   }

   if (AngleDeg < 180.0) {   //Angle between 0 and 180
      //Subtract 90 to shift range
      AngleDeg = AngleDeg -90.0;
      //Convert degree to radials
      AngleRad = (AngleDeg*3.141592)/180.0;

      CosA = -sin(AngleRad);   //Cos 0 to 180 deg = -sin(Angle Rad - 90deg)
      SinA = cos(AngleRad);      //Sin o to 180 deg = cos(Angle Rad - 90deg)
   }   
   else {   //Angle between 180 and 360
      //Subtract 270 to shift range
      AngleDeg = AngleDeg -270.0;
      //Convert degree to radials
      AngleRad = (AngleDeg*3.141592)/180.0;
      
      SinA = -cos(AngleRad);      //Sin 180 to 360 deg = -cos(Angle Rad - 270deg)
      CosA = sin(AngleRad);   //Cos 180 to 360 deg = sin(Angle Rad - 270deg)
   }
   

return;
}

/**--------------------------------------------------------------------
;[BODY INVERSE KINEMATICS]
;BodyRotX         - Global Input pitch of the body
;BodyRotY         - Global Input rotation of the body
;BodyRotZ         - Global Input roll of the body
;RotationY         - Input Rotation for the gait
;PosX            - Input position of the feet X
;PosZ            - Input position of the feet Z
;BodyOffsetX      - Input Offset betweeen the body and Coxa X
;BodyOffsetZ      - Input Offset betweeen the body and Coxa Z
;SinB                - Sin buffer for BodyRotX
;CosB              - Cos buffer for BodyRotX
;SinG                - Sin buffer for BodyRotZ
;CosG              - Cos buffer for BodyRotZ
;BodyIKPosX         - Output Position X of feet with Rotation
;BodyIKPosY         - Output Position Y of feet with Rotation
;BodyIKPosZ         - Output Position Z of feet with Rotation */
void BodyIK(signed int PosX, signed int PosZ, signed int PosY, signed int BodyOffsetX, signed int BodyOffsetZ, signed int RotationY) {
       
  //Calculating totals from center of the body to the feet
  TotalZ = BodyOffsetZ+PosZ;
  TotalX = BodyOffsetX+PosX;
  //PosY are equal to a "TotalY"
 
 
  //Successive global rotation matrix:
  //Math shorts for rotation: Alfa (A) = Xrotate, Beta (B) = Zrotate, Gamma (G) = Yrotate
  //Sinus Alfa = sinA, cosinus Alfa = cosA. and so on...
  
  //First calculate sinus and cosinus for each rotation:
  GetSinCos((float)(BodyRotX+TotalXBal));
  SinG = SinA;
  CosG = CosA;
  GetSinCos((float)(BodyRotZ+TotalZBal));
  SinB = SinA;
  CosB = CosA;
  GetSinCos((float)(BodyRotY+RotationY+TotalYBal));
  
  //Calcualtion of rotation matrix:
  BodyIKPosX = TotalX- (signed int)((float)(TotalX)*CosA*CosB - (float)(TotalZ)*CosB*SinA + (float)(PosY)*SinB);
  BodyIKPosZ = TotalZ- (signed int)((float)(TotalX)*CosG*SinA + (float)(TotalX)*CosA*SinB*SinG +(float)(TotalZ)*CosA*CosG-(float)(TotalZ)*SinA*SinB*SinG-(float)(PosY)*CosB*SinG);
  BodyIKPosY = PosY -  (signed int)((float)(TotalX)*SinA*SinG - (float)(TotalX)*CosA*CosG*SinB + (float)(TotalZ)*CosA*SinG + (float)(TotalZ)*CosG*SinA*SinB + (float)(PosY)*CosB*CosG);
 
       
  
return;
}


/*--------------------------------------------------------------------
;[BOOGTAN2] Gets the Inverse Tangus from X/Y with the where Y can be zero or negative
;BoogTanX       - Input X
;BoogTanY       - Input Y
;BoogTan        - Output BOOGTAN2(X/Y)*/
void GetBoogTan(float BoogTanX, float BoogTanY){
      

   if(BoogTanX == 0) {   // X=0 -> 0 or PI
      if(BoogTanY >= 0) {
         BoogTan = 0.0;
         //putchar('P');
      }else {
         BoogTan = 3.141592;
      }
   }else {
   
      if(BoogTanY == 0) {   // Y=0 -> +/- Pi/2
         if(BoogTanX > 0) {
            BoogTan = 3.141592 / 2.0;
         }else {
            BoogTan = -3.141592 / 2.0;
         }
      }else {
         
         if(BoogTanY > 0) {   //BOOGTAN(X/Y)
            BoogTan = atan((BoogTanX) / (BoogTanY));
            //putchar('M');
         }else  { 
            if(BoogTanX > 0) {   //BOOGTAN(X/Y) + PI   
               BoogTan = atan((BoogTanX) / (BoogTanY)) + 3.141592;
            }else {              //BOOGTAN(X/Y) - PI   
               BoogTan = atan((BoogTanX) / (BoogTanY)) - 3.141592;
            }
         }
      }
   }
   
  
   
return;
}

/*--------------------------------------------------------------------
;[LEG INVERSE KINEMATICS] Calculates the angles of the tibia and femur for the given position of the feet
;IKFeetPosX         - Input position of the Feet X
;IKFeetPosY         - Input position of the Feet Y
;IKFeetPosZ         - Input Position of the Feet Z
;IKSolution         - Output true IF the solution is possible
;IKSolutionWarning    - Output true IF the solution is NEARLY possible
;IKSolutionError   - Output true IF the solution is NOT possible
;IKFemurAngle      - Output Angle of Femur in degrees
;IKTibiaAngle      - Output Angle of Tibia in degrees
;IKCoxaAngle      - Output Angle of Coxa in degrees*/
void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ){
   
         
   //Length between the Coxa and Feet
   IKFeetPosXZ = (int) sqrt((float)((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

   //IKSW - Length between shoulder and wrist
   IKSW = sqrt((float)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
   //IKSW2 = sqrt((float)(((IKFeetPosXZ-CoxaLength)*(IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
      
   //IKA1 - Angle between SW line and the ground in rad
   GetBoogTan(IKFeetPosXZ-CoxaLength, IKFeetPosY);
   IKA1 = BoogTan;
   
   //IKA2 - ?
   IKA2 = acos( ((float)((FemurLength*FemurLength) - (TibiaLength*TibiaLength)) + (IKSW*IKSW)) / ((float)(2*FemurLength) * IKSW) );
   
   //IKFemurAngle
   IKFemurAngle = ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90;
   
   //IKTibiaAngle
   IKTibiaAngle = -( 90-(acos( ( (float)(FemurLength*FemurLength) + (TibiaLength*TibiaLength) - (IKSW*IKSW) ) / ((float)(2*FemurLength*TibiaLength)) )*180.0 / 3.141592) );
   
   //IKCoxaAngle
   GetBoogTan(IKFeetPosZ, IKFeetPosX);
   IKCoxaAngle = ((BoogTan*180.0) / 3.141592);
      
   //Set the Solution quality   
   if(IKSW < (float)(FemurLength+TibiaLength-30)) {
      IKSolution = TRUE;
   }
   else {
      if(IKSW < (float)(FemurLength+TibiaLength)) {
         IKSolutionWarning = TRUE;                 
      }
      else {
         IKSolutionError = TRUE;
      }   
   }      
    
    
        
return;
}


/*--------------------------------------------------------------------
;[LEG INVERSE KINEMATICS] Calculates the angles of the tibia and femur for the given position of the feet
;IKFeetPosX         - Input position of the Feet X
;IKFeetPosY         - Input position of the Feet Y
;IKFeetPosZ         - Input Position of the Feet Z
;IKSolution         - Output true IF the solution is possible
;IKSolutionWarning    - Output true IF the solution is NEARLY possible
;IKSolutionError   - Output true IF the solution is NOT possible
;IKFemurAngle      - Output Angle of Femur in degrees
;IKTibiaAngle      - Output Angle of Tibia in degrees
;IKCoxaAngle      - Output Angle of Coxa in degrees*/
void LegIK2(signed long IKFeetPosX, signed long IKFeetPosY, signed long IKFeetPosZ){
   
         
   //Length between the Coxa and Feet
   IKFeetPosXZ = (int) sqrt(((IKFeetPosX*IKFeetPosX)+(IKFeetPosZ*IKFeetPosZ)));

   //IKSW - Length between shoulder and wrist
   IKSW = sqrt((((long)((long)IKFeetPosXZ-CoxaLength)*(long)((long)IKFeetPosXZ-CoxaLength))+(IKFeetPosY*IKFeetPosY)));
      
   //IKA1 - Angle between SW line and the ground in rad
   GetBoogTan(IKFeetPosXZ-CoxaLength, IKFeetPosY);
   IKA1 = BoogTan;
   
   //IKA2 - ?
   IKA2 = acos( ((float)(((long)FemurLength2*(long)FemurLength2) - ((long)TibiaLength2*(long)TibiaLength2)) + (IKSW*IKSW)) / ((float)(2*FemurLength2) * IKSW) );
   
   //IKFemurAngle
   IKFemurAngle = ( -(IKA1 + IKA2) * 180.0 / 3.141592 )+90;
   
   //IKTibiaAngle
   IKTibiaAngle = -( 90-(acos( ( (float)(FemurLength2*FemurLength2) + (TibiaLength2*TibiaLength2) - (IKSW*IKSW) ) / ((float)(2*FemurLength2*TibiaLength2)) )*180.0 / 3.141592) );
   
   //IKCoxaAngle
   GetBoogTan(IKFeetPosZ, IKFeetPosX);
   IKCoxaAngle = ((BoogTan*180.0) / 3.141592);
      
   //Set the Solution quality   
   if(IKSW < (float)(FemurLength2+TibiaLength2-30)) {
      IKSolution = TRUE;
   }
   else {
      if(IKSW < (float)(FemurLength2+TibiaLength2)) {
         IKSolutionWarning = TRUE;                 
      }
      else {
         IKSolutionError = TRUE;
      }   
   }      
         
   
return;
}


signed int RotPoint = 0; 
void setRotPoint(signed int z) {

        RotPoint = z;

        return;
}

char mylegnumber = 0;


void firstposition(void) {


if(starting == 0) {

RFPosX = 60;      //Start positions of the Right Front leg
RFPosY = 25;
RFPosZ = -21;

RMPosX = 100;   //Start positions of the Right Middle leg
RMPosY = 25;
RMPosZ = 0;	

RRPosX = 53;    //Start positions of the Right Rear leg
RRPosY = 25;
RRPosZ = 41;

LFPosX = 60;      //Start positions of the Left Front leg
LFPosY = 25;
LFPosZ = -21;

LMPosX = 100;   //Start positions of the Left Middle leg
LMPosY = 25;
LMPosZ = 0;

LRPosX = 53;      //Start positions of the Left Rear leg
LRPosY = 25;
LRPosZ = 41;
RotPoint = 82; //(arriere du robot)

//Body Positions
BodyPosX = 0;
BodyPosY = 0;
BodyPosZ = 0;
BodyPosXint = 0;
BodyPosZint = 0;
BodyPosYint = 0;

//Body Rotations
BodyRotX = 0;
BodyRotY = 0;
BodyRotZ = 0;

// 178 / 275
X = 0;
Y = 0;
Z = 0;

Xbase = 200;
Ybase = 108;
Zbase = -80;
#ifdef WRITINGBOT
Xbase = -40;
Ybase = 72;
Zbase = 0;
#endif
starting++;

}
else if( starting == 1 ) {              

BodyPosYint = 110;
NomGaitSpeed = 500;
starting++;

}
else if(starting == 2) {
   GaitSelect();
   starting++;
}

return;
}



void doBodyRot(void) {
        // Degres : BodyRot
        signed int PosZ1;
        // Rotation suivant Y
        GetSinCos( (float)BodyRotY );
        // BodyPosZ = cos( BodyRotY ) * ( (RotPoint / cos( BodyRotY )) - RotPoint);
        PosZ1 = (signed int)( ((float)RotPoint) * ( 1.0 - CosA ));
        BodyPosX = BodyPosXint + (signed int)( ((float)RotPoint) *  SinA);
        
        // Rotation suivant X
        GetSinCos( (float)BodyRotX );  
        BodyPosZ = BodyPosZint + PosZ1 + (signed int)( ((float)RotPoint) * ( 1.0 - CosA ));
        BodyPosY = BodyPosYint - (signed int)( ((float)RotPoint) *  SinA);
        
        return;
}


/*--------------------------------------------------------------------

;[SERVO DRIVER] Updates the positions of the servos    */

void ServoDriver(void){

    char Serout[260]={0};
    int temp = 0;


 //Front Right leg
        
	//temp = (int)((left*LEG[x].Coxa.Angle +90)/0.10588238)+650+LEG[x].Coxa.Center-1500;    // 645MG
//        if ( temp > 2500 || temp < 500)
//            LEG[x].Coxa.Error = true;
//        temp = max(min(temp, 2500),500);
//        GotoXY(80,1+LEG[x].Coxa.Pin); printf("#%dP%d   ", LEG[x].Coxa.Pin, temp);
	temp = (int)( (float)(-RFCoxaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET8;
        sprintf(Serout, "%s #%dP%d", Serout, 8, temp);

	temp = (int)( (float)(-RFFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET9;
        sprintf(Serout, "%s #%dP%d", Serout, 9, temp);

	temp = (int)( (float)(-RFTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET10;
        sprintf(Serout, "%s #%dP%d", Serout, 10, temp);


  //Middle Right leg     
        
	temp = (int)( (float)(-RMCoxaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET4;
        sprintf(Serout, "%s #%dP%d", Serout, 4, temp);

	temp = (int)( (float)(-RMFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET5;
        sprintf(Serout, "%s #%dP%d", Serout, 5, temp);

	temp = (int)( (float)(-RMTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET6;
        sprintf(Serout, "%s #%dP%d", Serout, 6, temp);

  //Rear Right leg

	temp = (int)( (float)(-RRCoxaAngle + 90)/0.10588238 ) + 650 + SERVO_OFFSET0;
        sprintf(Serout, "%s #%dP%d", Serout, 0, temp);

        temp = (int)( (float)(-RRFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET1;
        sprintf(Serout, "%s #%dP%d", Serout, 1, temp);

        temp = (int)( (float)(-RRTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET2;
        sprintf(Serout, "%s #%dP%d", Serout, 2, temp);



  //Front Left leg

	temp = (int)( (float)(LFCoxaAngle + 90)/0.10588238 ) + 650 + SERVO_OFFSET24;
        sprintf(Serout, "%s #%dP%d", Serout, 24, temp);

        temp = (int)( (float)(LFFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET25;
        sprintf(Serout, "%s #%dP%d", Serout, 25, temp);

        temp = (int)( (float)(LFTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET26;
        sprintf(Serout, "%s #%dP%d", Serout, 26, temp);
        

  //Middle Left leg    

        
	temp = (int)( (float)(LMCoxaAngle + 90)/0.10588238 ) + 650 + SERVO_OFFSET20;
        sprintf(Serout, "%s #%dP%d", Serout, 20, temp);

        temp = (int)( (float)(LMFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET21;
        sprintf(Serout, "%s #%dP%d", Serout, 21, temp);

        temp = (int)( (float)(LMTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET22;
        sprintf(Serout, "%s #%dP%d", Serout, 22, temp);


  //Rear Left leg

        temp = (int)( (float)(LRCoxaAngle + 90)/0.10588238 ) + 650 + SERVO_OFFSET16;
        sprintf(Serout, "%s #%dP%d", Serout, 16, temp);

        temp = (int)( (float)(LRFemurAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET17;
        sprintf(Serout, "%s #%dP%d", Serout, 17, temp);

        temp = (int)( (float)(LRTibiaAngle +90)/0.10588238 ) + 650 + SERVO_OFFSET18;
        sprintf(Serout, "%s #%dP%d", Serout, 18, temp);

  /* Head */ 

	temp = (int)( (float)(headAngle +90)/0.10588238 ) + 650 + 0;
        sprintf(Serout, "%s #%dP%d", Serout, 30, temp);
        
  /* Turret */
        
	temp = (int) horizontal_turret;
        sprintf(Serout, "%s #%dP%d", Serout, 28, temp);
	temp = (int) vertical_turret;
        sprintf(Serout, "%s #%dP%d", Serout, 29, temp);


  // ARM

        /*

        entier = (unsigned int) abs((int)((-ARMCoxaAngle +90)/0.10588238) + 650 + 0);
        display4 = (entier % 10) +48;
        display3 = ((entier/10) % 10) +48; 
        display2 = ((entier/100) % 10) +48; 
        display1 = ((entier/1000) % 10) +48;
        putchar('#');
        putchar('2');// 1
        putchar('2');  
        putchar('P');
        if(entier > 999)
                putchar(display1);
        putchar(display2);
        putchar(display3);
        putchar(display4);
         
        entier = (unsigned int) abs((int)((-ARMFemurAngle +90)/0.10588238) + 650 + 0);
        display4 = (entier % 10) +48;
        display3 = ((entier/10) % 10) +48; 
        display2 = ((entier/100) % 10) +48; 
        display1 = ((entier/1000) % 10) +48;
        putchar('#');
        putchar('1');
        putchar('3');  
        putchar('P');
        if(entier > 999)
                putchar(display1);
        putchar(display2);
        putchar(display3);
        putchar(display4);
        
        entier = (unsigned int) abs((int)((ARMTibiaAngle +90)/0.10588238) + 650 + 0);
        display4 = (entier % 10) +48;
        display3 = ((entier/10) % 10) +48; 
        display2 = ((entier/100) % 10) +48; 
        display1 = ((entier/1000) % 10) +48;
        putchar('#');
        putchar('2');
        putchar('8');  
        putchar('P');
        if(entier > 999)
                putchar(display1);
        putchar(display2);
        putchar(display3);
        putchar(display4);
        */ 
  //Send <CR>
  /*putchar('T');
  entier = (unsigned int) NomGaitSpeed;
  display4 = (entier % 10) +48;
  display3 = ((entier/10) % 10) +48; 
  display2 = ((entier/100) % 10) +48;
  display1 = ((entier/1000) % 10) +48;
  if(entier > 999)
                putchar(display1);
  putchar(display2);
  putchar(display3);
  putchar(display4);
  putchar(13);
  //serout SSC_OUT,SSC_BAUTE,["T",dec SSCTime,13]
        */
  //putchar('\n');
  //PrevSSCTime = SSCTime

  // Time and <CR>
    sprintf(Serout, "%s T%d\r", Serout, NomGaitSpeed);

    // write to serial if connected
    if ( ser_fd_ssc )
        write(ser_fd_ssc, &Serout, sizeof(Serout));

	printf("%s \n",Serout);
return;
}


//--------------------------------------------------------------------
//[FREE SERVOS] Frees all the servos
void FreeServos()
{
    int x =0;
    char Serout[260]={0}, temp[10]={0};
    for ( x=0; x <= 31; x++ )
    {
        sprintf(Serout, "%s#%dP0 ", Serout, x);
    }
    sprintf(Serout, "%sT200\r", Serout);

    // write to serial if connected
    if ( ser_fd_ssc )
        write(ser_fd_ssc, &Serout, sizeof(Serout));
}



void getInput(void) {

char my_input[50];
char read_flag;
char Serout[260]={0}, temp[10]={0};

read_flag = read(ser_fd_modem, my_input, 1);;

	if( (read_flag != 0) || (my_input[0] != 0) ) {
		
		if(read_flag == -1){
			printf("Reading error.");
		}
		else {
			switch(my_input[0]) {
				case '8' : 
					TravelLengthZ -= 10;
					break;
				case '2' : 
                                        TravelLengthZ += 10;
                                        break;
				default : break;
			}
			printf("Char received = %x",my_input[0]);
		}

	}		

    sprintf(Serout, "Q\r");

    // write to serial if connected
    if ( ser_fd_ssc )
        write(ser_fd_ssc, &Serout, sizeof(Serout));



}




void main(void)
{
// Declare your local variables here
int timeout_end = 0;




// USART0 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART0 Receiver: On
// USART0 Transmitter: On
// USART0 Mode: Asynchronous
// USART0 Baud rate: 115200

ser_fd_ssc = open(SSCDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

if( ser_fd_ssc == -1)
    {
        printf( " SSC Serial Not Open \n" );
    }
    else
    {
        printf( " SSC Serial Open \n" );
        tcgetattr(ser_fd_ssc, &oldtio_ssc);                             // Backup old port settings
        memset(&newtio_ssc, 0, sizeof(newtio_ssc));

        newtio_ssc.c_iflag = IGNBRK | IGNPAR;
        newtio_ssc.c_oflag = 0;
        newtio_ssc.c_cflag = BAUDRATE | CREAD | CS8 | CLOCAL;
        newtio_ssc.c_lflag = 0;

        tcflush(ser_fd_ssc, TCIFLUSH);
        tcsetattr(ser_fd_ssc, TCSANOW, &newtio_ssc);

        memset(&newtio_ssc, 0, sizeof(newtio_ssc));
        tcgetattr(ser_fd_ssc, &newtio_ssc);

	fcntl(ser_fd_ssc, F_SETFL, FNDELAY);

    }




// USART1 initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART1 Receiver: On
// USART1 Transmitter: On
// USART1 Mode: Asynchronous
// USART1 Baud rate: 115200


ser_fd_modem = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);

if( ser_fd_modem == -1)
    {
        printf( " MODEM Serial Not Open \n" );
    }
    else
    {
        printf( " MODEM Serial Open \n" );
        fcntl(ser_fd_modem, F_SETFL, FNDELAY);
	tcgetattr(ser_fd_modem, &oldtio_modem);                             // Backup old port settings
        memset(&newtio_modem, 0, sizeof(newtio_modem));

        newtio_modem.c_iflag = IGNBRK | IGNPAR;
        newtio_modem.c_oflag = 0;
        newtio_modem.c_cflag = BAUDRATE | CREAD | CS8 | CLOCAL;
        newtio_modem.c_lflag = 0;

        tcflush(ser_fd_modem, TCIFLUSH);
        tcsetattr(ser_fd_modem, TCSANOW, &newtio_modem);

        memset(&newtio_modem, 0, sizeof(newtio_modem));
        tcgetattr(ser_fd_modem, &newtio_modem);
    
	fcntl(ser_fd_modem, F_SETFL, FNDELAY);
    }




// Global enable interrupts



//====================================================================
//[INIT]
 
horizontal_turret = 1500;
vertical_turret = 2150;

//Gait
GaitType = 1;
BalanceMode = 0;
LegLiftHeight = 50;
GaitStep = 1;
Mode = 1;

//What leg is active variable 1-6

//Whatleg = 0

//This resets the Init positions of each leg when they are modified with two leg mode
ResetInitPos = False;

//GaitSelect();




  BodyPosY = 0;
  TravelLengthX = 0;
  TravelLengthZ = 0;
  TravelRotationY = 0;
  
sleep(1);

system("aplay /home/root/sons_r2d2/r2d211.wav &");


GaitSelect();

#ifdef WRITINGBOT
indexarmtab = 0;
armtab[0][0] = 0;
armtab[0][1] = 0;
armtab[0][2] = 0;
#endif

while (1)
      {
      // Place your code here
      
  firstposition();   
      

//====================================================================
        //[MAIN]   

  //Start time
  //GOSUB GetCurrentTime[], lTimerStart
  //Read input
  //GOSUB RCInput1
  
  
  getInput();
  
  /*armWrite();*/                
 
  //Reset IKsolution indicators                                                                                                                                                
  IKSolution = False;
  IKSolutionWarning = False;
  IKSolutionError = False;
 
  //Gait
  GaitSeq();
   
  //Balance calculations
  TotalTransX = 0; //reset values used for calculation of balance
  TotalTransZ = 0;
  TotalTransY = 0;
  TotalXBal = 0;
  TotalYBal = 0;
  TotalZBal = 0;
  /*
  IF (BalanceMode>0) THEN 
   gosub BalCalcOneLeg [-RFPosX+BodyPosX+RFGaitPosX, RFPosZ+BodyPosZ+RFGaitPosZ,RFGaitPosY, RFOffsetX, RFOffsetZ]
   gosub BalCalcOneLeg [-RMPosX+BodyPosX+RMGaitPosX, RMPosZ+BodyPosZ+RMGaitPosZ,RMGaitPosY, RMOffsetX, RMOffsetZ]
   gosub BalCalcOneLeg [-RRPosX+BodyPosX+RRGaitPosX, RRPosZ+BodyPosZ+RRGaitPosZ,RRGaitPosY, RROffsetX, RROffsetZ]
   gosub BalCalcOneLeg [LFPosX-BodyPosX+LFGaitPosX, LFPosZ+BodyPosZ+LFGaitPosZ,LFGaitPosY, LFOffsetX, LFOffsetZ]
   gosub BalCalcOneLeg [LMPosX-BodyPosX+LMGaitPosX, LMPosZ+BodyPosZ+LMGaitPosZ,LMGaitPosY, LMOffsetX, LMOffsetZ]
   gosub BalCalcOneLeg [LRPosX-BodyPosX+LRGaitPosX, LRPosZ+BodyPosZ+LRGaitPosZ,LRGaitPosY, LROffsetX, LROffsetZ]
   gosub BalanceBody
  ENDIF
  */ 
  
  //Reset IKsolution indicators
  IKSolution = False;
  IKSolutionWarning = False;
  IKSolutionError = False;
  
  
  
  doBodyRot();
  
  //Right Front leg
  BodyIK(-RFPosX+BodyPosX+RFGaitPosX, RFPosZ+BodyPosZ+RFGaitPosZ,RFPosY+BodyPosY+RFGaitPosY, (signed int)RFOffsetX, (signed int)RFOffsetZ, (signed int)RFGaitRotY);
  LegIK(RFPosX-BodyPosX+BodyIKPosX-RFGaitPosX, RFPosY+BodyPosY-BodyIKPosY+RFGaitPosY, RFPosZ+BodyPosZ-BodyIKPosZ+RFGaitPosZ);   
  RFCoxaAngle  = IKCoxaAngle + CoxaAngle; //Angle for the basic setup for the front leg   
  RFFemurAngle = IKFemurAngle;
  RFTibiaAngle = IKTibiaAngle;
   
  //Right Middle leg
  BodyIK(-RMPosX+BodyPosX+RMGaitPosX, RMPosZ+BodyPosZ+RMGaitPosZ,RMPosY+BodyPosY+RMGaitPosY, (signed int)RMOffsetX, (signed int)RMOffsetZ, (signed int)RMGaitRotY);
  LegIK(RMPosX-BodyPosX+BodyIKPosX-RMGaitPosX, RMPosY+BodyPosY-BodyIKPosY+RMGaitPosY, RMPosZ+BodyPosZ-BodyIKPosZ+RMGaitPosZ);
  RMCoxaAngle  = IKCoxaAngle;
  RMFemurAngle = IKFemurAngle;
  RMTibiaAngle = IKTibiaAngle;  
   
  //Right Rear leg
  BodyIK(-RRPosX+BodyPosX+RRGaitPosX, RRPosZ+BodyPosZ+RRGaitPosZ,RRPosY+BodyPosY+RRGaitPosY, (signed int)RROffsetX, (signed int)RROffsetZ, (signed int)RRGaitRotY);
  LegIK(RRPosX-BodyPosX+BodyIKPosX-RRGaitPosX, RRPosY+BodyPosY-BodyIKPosY+RRGaitPosY, RRPosZ+BodyPosZ-BodyIKPosZ+RRGaitPosZ);
  RRCoxaAngle  = IKCoxaAngle - CoxaAngle; //Angle for the basic setup for the front leg   
  RRFemurAngle = IKFemurAngle;
  RRTibiaAngle = IKTibiaAngle;

  //Left Front leg
  BodyIK(LFPosX-BodyPosX+LFGaitPosX, LFPosZ+BodyPosZ+LFGaitPosZ,LFPosY+BodyPosY+LFGaitPosY, (signed int)LFOffsetX, (signed int)LFOffsetZ, (signed int)LFGaitRotY);
  LegIK(LFPosX+BodyPosX-BodyIKPosX+LFGaitPosX, LFPosY+BodyPosY-BodyIKPosY+LFGaitPosY, LFPosZ+BodyPosZ-BodyIKPosZ+LFGaitPosZ);
  LFCoxaAngle  = IKCoxaAngle + CoxaAngle; //Angle for the basic setup for the front leg   
  LFFemurAngle = IKFemurAngle;
  LFTibiaAngle = IKTibiaAngle;

  //Left Middle leg
  BodyIK(LMPosX-BodyPosX+LMGaitPosX, LMPosZ+BodyPosZ+LMGaitPosZ,LMPosY+BodyPosY+LMGaitPosY, (signed int)LMOffsetX, (signed int)LMOffsetZ, (signed int)LMGaitRotY);
  LegIK(LMPosX+BodyPosX-BodyIKPosX+LMGaitPosX, LMPosY+BodyPosY-BodyIKPosY+LMGaitPosY, LMPosZ+BodyPosZ-BodyIKPosZ+LMGaitPosZ);
  LMCoxaAngle  = IKCoxaAngle;
  LMFemurAngle = IKFemurAngle;
  LMTibiaAngle = IKTibiaAngle;
         
  //Left Rear leg
  BodyIK(LRPosX-BodyPosX+LRGaitPosX, LRPosZ+BodyPosZ+LRGaitPosZ,LRPosY+BodyPosY+LRGaitPosY, (signed int)LROffsetX, (signed int)LROffsetZ, (signed int)LRGaitRotY);
  LegIK(LRPosX+BodyPosX-BodyIKPosX+LRGaitPosX, LRPosY+BodyPosY-BodyIKPosY+LRGaitPosY, LRPosZ+BodyPosZ-BodyIKPosZ+LRGaitPosZ);
  LRCoxaAngle  = IKCoxaAngle - CoxaAngle; //Angle for the basic setup for the front leg   
  LRFemurAngle = IKFemurAngle;
  LRTibiaAngle = IKTibiaAngle;
 
   /*
   CheckAngles();
   putchar('Q');
   putchar(13);
   timeout_end = 0;
   while( getchar() != '.' ){
        timeout_end++;
        if( timeout_end > 400 )
                break;
        delay_ms(5);
        putchar('Q');
        putchar(13);
   }
*/ 

   usleep(NomGaitSpeed*1000);

   ServoDriver();
   
      
      };
}
