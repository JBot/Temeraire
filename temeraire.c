
#include "temeraire.h"

#define NEW_INPUT_PROTOCOL
#define FOOT_SENSORS


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

// Legs sensors
char leg_on_floor = 1;
#define DOWN_SENSOR_SPEED 20

/* Functions */
// I/O Functions
void open_interfaces(void);
int read_reg_i2c(int fh, uint8_t reg, int count);
int write_reg_i2c(int fh, uint8_t reg, uint8_t val);
float read_adc(int file, struct twl4030_madc_user_parms *par, int adc_channel);

// Robot Gait and IK functions
void GaitSelect(void);
void Gait(char GaitLegNr, signed int GaitPosXX, signed int GaitPosYY, signed int GaitPosZZ, signed int GaitRotYY);
void GaitSeq(void);
void GetSinCos (float AngleDeg);
void BodyIK(signed int PosX, signed int PosZ, signed int PosY, signed int BodyOffsetX, signed int BodyOffsetZ, signed int RotationY);
void GetBoogTan(float BoogTanX, float BoogTanY);
void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);
void LegIK2(signed long IKFeetPosX, signed long IKFeetPosY, signed long IKFeetPosZ);
void setRotPoint(signed int z);
void firstposition(void);
void doBodyRot(void);

// Servo and Input to give orders
void ServoDriver(void);
void FreeServos();
#ifdef NEW_INPUT_PROTOCOL
void *getInput(void *args);
#else 
void getInput(void);
#endif

// Balance functions
void BalCalcOneLeg(double my_PosX, double my_PosZ,double my_PosY,int my_BodyOffsetX, int my_BodyOffsetZ);
void BalanceBody();

// Threads
void *thread_adc_battery(void *arg);
void *thread_i2c_ultrasound(void *args);




void *thread_i2c_ultrasound(void *arg) {
	uint16_t result, result2;
	fprintf(stdout, "Thread: %d (Ultrasound sensor) running.\n", (int)2);
	usleep(70000);
	while(1) {

		if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
			printf("\n\nwrite bad...\n");

		usleep(100000);

		result = (uint16_t)read_reg_i2c(file_i2c, 0x02, 1);
		result2 = (uint16_t)read_reg_i2c(file_i2c, 0x03, 1);
		/*printf("1 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		  result = (uint16_t)read_reg_i2c(file_i2c, 0x04, 1);
		  result2 = (uint16_t)read_reg_i2c(file_i2c, 0x05, 1);
		  printf("2 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		  result = (uint16_t)read_reg_i2c(file_i2c, 0x06, 1);
		  result2 = (uint16_t)read_reg_i2c(file_i2c, 0x07, 1);
		  printf("3 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		 */
		us_sensor_light = (uint16_t)read_reg_i2c(file_i2c, 0x01, 1);		

		if (result == -1)
			printf("\n\nread bad...\n");

		//printf("IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		us_sensor_distance = ((result & 0x00FF) << 8) + (result2 &  0x00FF);
		//printf("us_sensor : %i \n", us_sensor_distance);

	}
	fprintf(stdout, "Thread: %d done.\n", (int)2);

	pthread_exit(0);
}


int read_reg_i2c(int fh, uint8_t reg, int count) {
	uint8_t data[2];

	if (count < 0 || count > 2)
		return -1;

	data[0] = reg;

	if (write(fh, &data, 1) != 1) {
		perror("write before read");
		return -1;
	}

	data[1] = 0;

	if (read(fh, &data, count) != count) {
		perror("read");
		return -1;
	}

	return (data[1] << 8) + data[0];
}

int write_reg_i2c(int fh, uint8_t reg, uint8_t val) {
	uint8_t data[2];

	data[0] = reg;
	data[1] = val;

	if (write(fh, &data, 2) != 2) {
		perror("write");
		return -1;
	}

	return 1;
}


float read_adc(int file, struct twl4030_madc_user_parms *par, int adc_channel) {

	memset(par, 0, sizeof(struct twl4030_madc_user_parms));

	par->channel = channels[adc_channel].number;
	int ret = ioctl(file, TWL4030_MADC_IOCX_ADC_RAW_READ, par);
	float result = ((unsigned int)par->result) / 1024.f; // 10 bit ADC -> 1024

	if (ret == 0 && par->status != -1)
		printf("%s (channel %d): %f\n", channels[adc_channel].name,
				channels[adc_channel].number, result * channels[adc_channel].input_range);
	else
	{
		if (par->status == -1)
			printf("Channel %d (%s) timed out!\n", adc_channel, channels[adc_channel].name);
		printf("ERROR\n");
	}

	return result;

}



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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
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
		NomGaitSpeed = 250;
	}

	if(GaitType == 8) { // TEST
		LRGaitLegNr = 3;
		RFGaitLegNr = 1;
		LMGaitLegNr = 11;
		RRGaitLegNr = 9;
		LFGaitLegNr = 7;
		RMGaitLegNr = 5;

		NrLiftedPos = 1;
		HalfLiftHeigth = FALSE;
		TLDivFactor = 10;
		StepsInGait = 12;
		NomGaitSpeed = 250;
	}


	if(GaitType == 9) { // 4 legs
		LRGaitLegNr = 1; 
		//RMGaitLegNr = 21;
		//LMGaitLegNr = 4;
		RRGaitLegNr = 13;

		LFGaitLegNr = 9;
		RFGaitLegNr = 21;

		NrLiftedPos = 3;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 20;    
		StepsInGait = 24;      
		NomGaitSpeed = 150;
	}

	if(GaitType == 10) { //Wave 18 steps
		LRGaitLegNr = 1; 
		RFGaitLegNr = 2; 
		LMGaitLegNr = 8; 
		RRGaitLegNr = 15; 
		LFGaitLegNr = 12; 
		RMGaitLegNr = 18; 

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 18;    
		StepsInGait = 20;      
		NomGaitSpeed = 400;
	}

	if(GaitType == 11) { //Wave 14 steps
		LRGaitLegNr = 5; 
		RFGaitLegNr = 2;
		RRGaitLegNr = 12; 
		LFGaitLegNr = 9; 

		RMGaitLegNr = 16;
		LMGaitLegNr = 20;

		NrLiftedPos = 2;
		HalfLiftHeigth = FALSE;   
		TLDivFactor = 12;    
		StepsInGait = 14;      
		NomGaitSpeed = 400;
	}

	ActualGaitSpeed = NomGaitSpeed;

	return;
}

#ifdef FOOT_SENSORS
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
					if ((GaitStep==GaitLegNr+NrLiftedPos || GaitStep==GaitLegNr-(StepsInGait-NrLiftedPos))) {         

						if(GaitLegNr == LFGaitLegNr) { // uniquement la patte avant gauche pour l'instant
							leg_on_floor = 0;
							file_gpio146 = open("/sys/class/gpio/gpio146/value", O_RDWR | O_NONBLOCK);
							read(file_gpio146, gpio146_input, 1);
							close(file_gpio146);
							printf("gpio146_input[0] = %d \n",gpio146_input[0]);
							if(gpio146_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								GaitPosX = TravelLengthX/2;
                                                                GaitPosZ = TravelLengthZ/2;
                                                                GaitRotY = TravelRotationY/2;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg

								GaitPosX = TravelLengthX/2;
								GaitPosY = GaitPosYY + down_leg_step;
								GaitPosZ = TravelLengthZ/2;
								GaitRotY = TravelRotationY/2;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}

						}
						else {



							if(GaitLegNr == RFGaitLegNr) { // uniquement la patte avant droite pour l'instant
								leg_on_floor = 0;
								file_gpio147 = open("/sys/class/gpio/gpio147/value", O_RDWR | O_NONBLOCK);
								read(file_gpio147, gpio147_input, 1);
								close(file_gpio147);
								printf("gpio147_input[0] = %d \n",gpio147_input[0]);
								if(gpio147_input[0] == 48) { // leg on the floor
									leg_on_floor = 1;
									printf("GaitPosY = %d \n",GaitPosY);
									GaitPosY = GaitPosYY - 3;
									GaitPosX = TravelLengthX/2;
									GaitPosZ = TravelLengthZ/2;
									GaitRotY = TravelRotationY/2;
									// remettre la vitesse normale
									ActualGaitSpeed = NomGaitSpeed;
								}
								else { // must down the leg

									GaitPosX = TravelLengthX/2;
									GaitPosY = GaitPosYY + down_leg_step;
									GaitPosZ = TravelLengthZ/2;
									GaitRotY = TravelRotationY/2;
									// Mettre une vitesse rapide pour descendre la patte
									ActualGaitSpeed = DOWN_SENSOR_SPEED;
								}

							}


							else { // Pour les autres pattes, fonctionnement normal


								GaitPosX = TravelLengthX/2;
								GaitPosY = GaitPosYY + LegLiftHeight/((signed int)HalfLiftHeigth+1);//down_leg_step;
								GaitPosZ = TravelLengthZ/2;
								GaitRotY = TravelRotationY/2;
								//printf("Leg go down \n");
							}
						} 

					}
					//Move body forward     
					else {

						if(leg_on_floor == 0) { // Leg not on the floor
							GaitPosX = GaitPosXX;
							GaitPosY = GaitPosYY;
							GaitPosZ = GaitPosZZ;
							GaitRotY = GaitRotYY;
						}
						else {
							GaitPosX = GaitPosXX - (TravelLengthX/TLDivFactor);     
							GaitPosY = GaitPosYY;
							GaitPosZ = GaitPosZZ - (TravelLengthZ/TLDivFactor);
							GaitRotY = GaitRotYY - (TravelRotationY/TLDivFactor);
						}
						//printf("Move body \n");
					}
				}
			}
		}
	}
	/*
	   printf("leg_on_floor = %d \n",leg_on_floor);
	   printf("GaitStep = %d \n",GaitStep); 
	 */
	//Advance to the next step !!!!!!!! TO CHANGE !!!!!!!!
	if (LastLeg && (leg_on_floor == 1)) {   //The last leg in this step and legs are on the floor
		GaitStep = GaitStep+1;
		if (GaitStep>StepsInGait) {
			GaitStep = 1;
		}
		//printf(" \n");
	}

	return;

}

#else 

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
						//printf("Leg go down \n");
					}
					//Move body forward     
					else {

						GaitPosX = GaitPosXX - (TravelLengthX/TLDivFactor);     
						GaitPosY = 0;
						GaitPosZ = GaitPosZZ - (TravelLengthZ/TLDivFactor);
						GaitRotY = GaitRotYY - (TravelRotationY/TLDivFactor);
						//printf("Move body \n");
					}
				}
			}

		}
	}
	/*
	   printf("leg_on_floor = %d \n",leg_on_floor);
	   printf("GaitStep = %d \n",GaitStep); 
	 */
	//Advance to the next step 
	if (LastLeg) {   //The last leg in this step
		GaitStep = GaitStep+1;
		if (GaitStep>StepsInGait) {
			GaitStep = 1;
		}
		//printf(" \n");
	}

	return;

}

#endif

/**--------------------------------------------------------------------
  [GAIT Sequence]*/
void GaitSeq(void) {
	//Calculate Gait sequence

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

	Gait(LMGaitLegNr, LMGaitPosX, LMGaitPosY, LMGaitPosZ, LMGaitRotY);
	LMGaitPosX = GaitPosX;
	LMGaitPosY = GaitPosY;
	LMGaitPosZ = GaitPosZ;
	LMGaitRotY = GaitRotY;   

	Gait(RRGaitLegNr, RRGaitPosX, RRGaitPosY, RRGaitPosZ, RRGaitRotY);
	RRGaitPosX = GaitPosX;
	RRGaitPosY = GaitPosY;
	RRGaitPosZ = GaitPosZ;
	RRGaitRotY = GaitRotY;   

	Gait(LFGaitLegNr, LFGaitPosX, LFGaitPosY, LFGaitPosZ, LFGaitRotY);
	LFGaitPosX = GaitPosX;
	LFGaitPosY = GaitPosY;
	LFGaitPosZ = GaitPosZ;
	LFGaitRotY = GaitRotY;   
	printf("LFGaitPosY = %d \n",LFGaitPosY);  

	LastLeg = TRUE;
	Gait(RMGaitLegNr, RMGaitPosX, RMGaitPosY, RMGaitPosZ, RMGaitRotY);
	RMGaitPosX = GaitPosX;
	RMGaitPosY = GaitPosY;
	RMGaitPosZ = GaitPosZ;
	RMGaitRotY = GaitRotY;

	printf("leg_on_floor = %d \n",leg_on_floor);
	printf("GaitStep = %d \n",GaitStep);   
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

		/* VRAIE POSITION */ 
		RFPosX = 60;      //Start positions of the Right Front leg
		RFPosY = 25;
		RFPosZ = -81;

		RMPosX = 90;   //Start positions of the Right Middle leg
		RMPosY = 25;
		RMPosZ = -10;	

		RRPosX = 73;    //Start positions of the Right Rear leg
		RRPosY = 25;
		RRPosZ = 61;

		LFPosX = 60;      //Start positions of the Left Front leg
		LFPosY = 25;
		LFPosZ = -81;

		LMPosX = 90;   //Start positions of the Left Middle leg
		LMPosY = 25;
		LMPosZ = -10;

		LRPosX = 73;      //Start positions of the Left Rear leg
		LRPosY = 25;
		LRPosZ = 61;


		/*
		   RFPosX = 30;      //Start positions of the Right Front leg
		   RFPosY = 25;
		   RFPosZ = 0;

		   RMPosX = 100;   //Start positions of the Right Middle leg
		   RMPosY = 0;
		   RMPosZ = 0;	

		   RRPosX = 33;    //Start positions of the Right Rear leg
		   RRPosY = 25;
		   RRPosZ = 0;

		   LFPosX = 30;      //Start positions of the Left Front leg
		   LFPosY = 25;
		   LFPosZ = 0;

		   LMPosX = 100;   //Start positions of the Left Middle leg
		   LMPosY = 0;
		   LMPosZ = 0;

		   LRPosX = 33;      //Start positions of the Left Rear leg
		   LRPosY = 25;
		   LRPosZ = 0;
		 */

		// RotPoint = 82; //(arriere du robot)
		RotPoint = 0; // (center du robot)

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
		ActualGaitSpeed = NomGaitSpeed;
	}
	else if( starting == 1 ) {              

		BodyPosYint = 100;
		NomGaitSpeed = 500;
		starting++;
		ActualGaitSpeed = NomGaitSpeed;
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

	/*
	// Head  
	// 3 DOFs
	temp = (int) ( (float)(BodyRotZ*7 + 90)/0.10588238 ) + 650;
	sprintf(Serout, "%s #%dP%d", Serout, 28, temp);
	temp = (int) ( (float)(-BodyRotY*4 + 90)/0.10588238 ) + 650;
	sprintf(Serout, "%s #%dP%d", Serout, 29, temp);
	temp = (int) ( (float)(-BodyRotX*4 + 90)/0.10588238 ) + 650;
	sprintf(Serout, "%s #%dP%d", Serout, 30, temp);
	// Mandibles
	temp = (int) Mandible;
	sprintf(Serout, "%s #%dP%d", Serout, 12, temp);
	sprintf(Serout, "%s #%dP%d", Serout, 13, 3000-temp);
	 */
	/*
	   temp = (int)( (float)(headAngle +90)/0.10588238 ) + 650 + 0;
	   sprintf(Serout, "%s #%dP%d", Serout, 30, temp);
	 */        
	/* Turret */
	/*      
		temp = (int) horizontal_turret;
		sprintf(Serout, "%s #%dP%d", Serout, 28, temp);
		temp = (int) vertical_turret;
		sprintf(Serout, "%s #%dP%d", Serout, 29, temp);
	 */

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
	sprintf(Serout, "%s T%d\r", Serout, ActualGaitSpeed);

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
	char Serout[260]={0};
	for ( x=0; x <= 31; x++ )
	{
		sprintf(Serout, "%s#%dP0 ", Serout, x);
	}
	sprintf(Serout, "%sT200\r", Serout);

	// write to serial if connected
	if ( ser_fd_ssc )
		write(ser_fd_ssc, &Serout, sizeof(Serout));
}

#ifdef NEW_INPUT_PROTOCOL
void *getInput(void *args) {

	char my_input[50];
	char read_flag;
	char Serout[260]={0};
	int i=0;
	char speak_buff[300]={0};

	printf("Thread Input running.\n");
	while(1) {
		usleep(8000);

		/* New test of comm protocol */

		if( wait_command_flag == 1) {
			/* Waiting command */

			read_flag = read(ser_fd_modem, my_input, 1);

			if( (read_flag != 0) && (my_input[0] != 0) ) {

				if(read_flag == -1){
					printf("Reading error.");
				}
				else {
					modem_command = my_input[0];
					wait_command_flag = 0;
					temp_input = 0;
					printf("Command received \n");
				}
			}
		}


		if( wait_command_flag == 0) {
			/* Waiting data */

			read_flag = read(ser_fd_modem, my_input, 1);

			if( (read_flag != 0) && (my_input[0] != 0) ) {

				if(read_flag == -1){
					printf("Reading error.");
				}
				else {

					if( my_input[0] == '\n' ) { // End of datas

						switch(modem_command) {

							case 'A' : // Balance mode
								BalanceMode = temp_input;
								break;
							case 'a' : // Balance mode

								break;
							case 'B' : // BodyPosX
								BodyPosXint = temp_input;
								break;
							case 'b' : 
								BodyPosXint = temp_input;
								break;
							case 'C' : // BodyPosY
								BodyPosYint = temp_input;
								break;
							case 'c' : 
								BodyPosYint = temp_input;
								break;
							case 'D' : // BodyPosZ
								BodyPosZint = temp_input;
								break;
							case 'd' : 
								BodyPosZint = temp_input;
								break;
							case 'E' : // BodyRotX
								BodyRotX = temp_input;
								break;
							case 'e' : 
								BodyRotX = temp_input;
								break;
							case 'F' : // BodyRotY
								BodyRotY = temp_input;
								break;
							case 'f' : 
								BodyRotY = temp_input;
								break;
							case 'G' : // BodyRotZ
								BodyRotZ = temp_input;
								break;
							case 'g' : 
								BodyRotZ = temp_input;
								break;
							case 'H' : // GaitType
								GaitType = temp_input;
								GaitSelect();
								break;
							case 'h' : 

								break;
							case 'I' : // GaitSpeed
#ifdef FOOT_SENSORS
								NomGaitSpeed = temp_input;
#else 
								ActualGaitSpeed = temp_input;
#endif
								break;
							case 'i' : 
								NomGaitSpeed = temp_input;
								break;
							case 'J' : // Mandibles
								Mandible = temp_input;
								break;
							case 'j' :
								Mandible = temp_input;
								break;
							case 'O' : // ON / OFF
								sleeping = 0;
								starting = 0;
								system("aplay /var/sons_r2d2/r2d25.wav &");
								break;
							case 'o' : // OFF
								FreeServos();
								TravelRotationY=0;
								TravelLengthZ = 0;
								TravelLengthX = 0;
								sleeping = 1;
								BodyPosYint = 0;
								system("aplay /var/sons_r2d2/r2d28.wav &");
								break;
							case 'R' : // RotationPoint
								RotPoint = temp_input;
								break;
							case 'r' : 
								RotPoint = temp_input;
								break;
							case 'S' : // Speak
								Serout[i++]='\0';
								//printf("%s",Serout);
								sprintf(speak_buff,"espeak -v fr \"%s\" &",Serout);
								system(speak_buff);
								i = 0;
								break;
							case 's' :
								Serout[i++]='\0';
								//printf("%s",Serout);
								sprintf(speak_buff,"espeak -v en \"%s\" &",Serout);
								system(speak_buff);
								i = 0;
								break;
							case 'T' : // TravelLengthX
								TravelLengthX = temp_input;
								break;
							case 't' : 
								TravelLengthX = temp_input;
								break;
							case 'U' : //TravelRotY
								//printf("Positif \n");
								TravelRotationY = temp_input;
								break;
							case 'u' : 
								//printf("Negatif \n");
								TravelRotationY = temp_input;
								break;
							case 'V' : // TravelLengthZ
								TravelLengthZ = temp_input;
								break;
							case 'v' : 
								TravelLengthZ = temp_input;
								break;
							case 'Z' : // Special commands

								break;
							case 'z' : 

								break;


							default : break;			


						}
						wait_command_flag = 1;

						temp_input = 0;
					}
					else { // New data

						if( (modem_command == 'S') || (modem_command == 's')) { // Speak command, so don't modify characters

							Serout[i++]=my_input[0];

						}
						else {
							if( modem_command < 92 )
								temp_input = (temp_input * 10) + (my_input[0] - 48);
							else 
								temp_input = (temp_input * 10) - (my_input[0] - 48);
						}
						//printf(" temp_input = %d \n", temp_input);
					}
				}
			}
		}
	}
	printf("Thread Input ended.\n");
}

#else

void getInput(void) {

	char my_input[50];
	char read_flag;
	char Serout[260]={0};

	read_flag = read(ser_fd_modem, my_input, 1);

	if( (read_flag != 0) || (my_input[0] != 0) ) {

		if(read_flag == -1){
			printf("Reading error.");
		}
		else {

			switch(my_input[0]) {

				case 'v' :
					RotPoint = -82; //(avant du robot)
					break; 
				case 'b' :
					RotPoint = 0; //(milieu du robot)
					break;        
				case 'n' :
					RotPoint = 82; //(arriere du robot)
					break;
				case 'B' :
					if(BalanceMode == 0){
						BalanceMode = 1;
						printf("Balance Mode ON  \n");
					}
					else {
						BalanceMode = 0;
						printf("Balance Mode OFF \n");
					}
					break;
				case 'w' :
					BodyPosZint += 5;
					break;        
				case 'x' :
					BodyPosZint -= 5;
					break;        
				case 'z' :
					BodyRotX += 5;
					break;        
				case 's' :
					BodyRotX -= 5;
					break;
				case 'q' :
					BodyPosXint += 5;
					break;
				case 'd' :
					BodyPosXint -= 5;
					break;        
				case '1' :
					TravelRotationY += 10;
					break;        
				case '2' :
					TravelLengthZ += 10;
					break;        
				case '3' :
					TravelRotationY -= 10;
					break;        
				case '4' :
					TravelRotationY += 10;
					break;        
				case '5' :
					FreeServos();
					TravelRotationY=0;
					TravelLengthZ = 0;
					TravelLengthX = 0;
					sleeping = 1;
					BodyPosYint = 0;
					system("aplay /var/sons_r2d2/r2d28.wav &");
					break;        
				case '6' :
					TravelRotationY -= 10;
					break;        
				case '7' :
					TravelLengthX += 10;
					break;        
				case '8' :
					TravelLengthZ -= 10;
					break;        
				case '9' :
					TravelLengthX -= 10;
					break;        
				case '0' :
					TravelRotationY=0;
					TravelLengthZ = 0;
					TravelLengthX = 0;
					break;        
				case '+' :
					NomGaitSpeed = NomGaitSpeed + 20;
					break;                
				case '-' :
					NomGaitSpeed = NomGaitSpeed - 20;
					break;                
				case '/' :
					if( GaitType == 0)
						GaitType = 8;
					else      
						GaitType--;
					GaitSelect();
					break;
				case '*' :
					if( GaitType == 8)
						GaitType = 0;
					else
						GaitType++;
					GaitSelect();
					break;
				case 'p' :
					sleeping = 0;
					starting = 0;
					system("aplay /var/sons_r2d2/r2d25.wav &");
					break;
				case 'm' :
					sprintf(Serout, "%s US SENSOR : %d ", Serout, us_sensor_distance);
					sprintf(Serout, "%s LIGHT SENSOR : %d \n\r", Serout, us_sensor_light);
					break;

				default : break;
			}

			printf("Char received = %x \n",my_input[0]);


			sprintf(Serout, "%s %d %d %d", Serout, TravelLengthX, TravelRotationY, TravelLengthZ);
			sprintf(Serout, "%s %d %d %d", Serout, BodyPosXint, BodyPosYint, BodyPosZint);
			sprintf(Serout, "%s %d %d %d", Serout, BodyRotX, BodyRotY, BodyRotZ);
			sprintf(Serout, "%s %d %d %d", Serout, GaitType, NomGaitSpeed, RotPoint);/*
												    sprintf(Serout, "%s %d %d %d", Serout, , , );
												    sprintf(Serout, "%s %d %d %d", Serout, , , );
												  */
			sprintf(Serout, "%s \n\r", Serout);
			// write to serial if connected
			if ( ser_fd_modem )
				write(ser_fd_modem, &Serout, sizeof(Serout));



		}

	}		




}
#endif

//;--------------------------------------------------------------------
//;[BalCalcOneLeg]
void BalCalcOneLeg(double my_PosX, double my_PosZ,double my_PosY,int my_BodyOffsetX, int my_BodyOffsetZ) {
	//;Calculating totals from center of the body to the feet
	TotalZ = my_BodyOffsetZ+my_PosZ;
	TotalX = my_BodyOffsetX+my_PosX;
	TotalY =  100 + my_PosY; //' using the value 150 to lower the centerpoint of rotation 'BodyPosY +
	TotalTransY = TotalTransY + my_PosY;
	TotalTransZ = TotalTransZ + TotalZ;
	TotalTransX = TotalTransX + TotalX;
	GetBoogTan( TotalX, TotalZ);
	TotalYBal = TotalYBal + (int)((BoogTan*180.0) / 3.141592);
	GetBoogTan (TotalX, TotalY);
	TotalZBal = TotalZBal + (int)((BoogTan*180.0) / 3.141592);
	GetBoogTan (TotalZ, TotalY);
	TotalXBal = TotalXBal + (int)((BoogTan*180.0) / 3.141592);
	return;
}

//;--------------------------------------------------------------------
//;[BalanceBody]
void BalanceBody() {
	TotalTransZ = TotalTransZ/6;
	TotalTransX = TotalTransX/6;
	TotalTransY = TotalTransY/6;
	if( TotalYBal < -180 )	//'Tangens fix caused by +/- 180 deg
		TotalYBal = TotalYBal + 360;
	if( TotalZBal < -180 )	//'Tangens fix caused by +/- 180 deg
		TotalZBal = TotalZBal + 360;
	if( TotalXBal < -180 )	//'Tangens fix caused by +/- 180 deg
		TotalXBal = TotalXBal + 360;

	//;Balance rotation
	TotalYBal = TotalYBal/6;
	TotalXBal = TotalXBal/6;
	TotalZBal = -TotalZBal/6;

	//;Balance translation
	LFGaitPosZ = LFGaitPosZ - TotalTransZ;
	LMGaitPosZ = LMGaitPosZ - TotalTransZ;
	LRGaitPosZ = LRGaitPosZ - TotalTransZ;
	RFGaitPosZ = RFGaitPosZ - TotalTransZ;
	RMGaitPosZ = RMGaitPosZ - TotalTransZ;
	RRGaitPosZ = RRGaitPosZ - TotalTransZ;

	LFGaitPosX = LFGaitPosX - TotalTransX;
	LMGaitPosX = LMGaitPosX - TotalTransX;
	LRGaitPosX = LRGaitPosX - TotalTransX;
	RFGaitPosX = RFGaitPosX - TotalTransX;
	RMGaitPosX = RMGaitPosX - TotalTransX;
	RRGaitPosX = RRGaitPosX - TotalTransX ;

	LFGaitPosY = LFGaitPosY - TotalTransY;
	LMGaitPosY = LMGaitPosY - TotalTransY;
	LRGaitPosY = LRGaitPosY - TotalTransY;
	RFGaitPosY = RFGaitPosY - TotalTransY;
	RMGaitPosY = RMGaitPosY - TotalTransY;
	RRGaitPosY = RRGaitPosY - TotalTransY;
	return;
}


void *thread_adc_battery(void *arg) {
	float battery_voltage = 0.0;
	fprintf(stdout, "Thread: %d running.\n", (int)1);

	while(1) {

		sleep((int)60);

		battery_voltage = read_adc(file_adc, par, 0);
		if( battery_voltage < 1.7  ) {
			system("aplay /var/sons_robot/low_battery.wav &");
			fprintf(stdout, "LOW BATTERY.\n");
		}

	}
	fprintf(stdout, "Thread: %d done.\n", (int)1);

	pthread_exit(0);
}



void open_interfaces(void) {

	float battery_voltage = 0.0;
	int thread_nb = 0;

	// USART0 initialization
	// Communication Parameters: 8 Data, 1 Stop, No Parity
	// USART0 Receiver: On
	// USART0 Transmitter: On
	// USART0 Mode: Asynchronous
	// USART0 Baud rate: 115200

	ser_fd_ssc = open(SSCDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
	system("aplay /var/sons_robot/serial_connection.wav ");
	if( ser_fd_ssc == -1)
	{
		printf( " SSC Serial Not Open \n" );
		system("aplay /var/sons_robot/nok.wav ");
	}
	else
	{
		printf( " SSC Serial Open \n" );
		system("aplay /var/sons_robot/ok.wav ");
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
	system("aplay /var/sons_robot/bluetooth_connection.wav ");
	if( ser_fd_modem == -1)
	{
		printf( " MODEM Serial Not Open \n" );
		system("aplay /var/sons_robot/nok.wav ");
	}
	else
	{
		printf( " MODEM Serial Open \n" );
		system("aplay /var/sons_robot/ok.wav ");
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


	/* US sensor check */
	file_i2c = open("/dev/i2c-3", O_RDWR);
	system("aplay /var/sons_robot/i2c_connection.wav ");
	if (file_i2c < 0) {
		printf("Could not open i2c-3\n");
		system("aplay /var/sons_robot/nok.wav ");
	}
	else { // i2c ok => list the available services
		system("aplay /var/sons_robot/ok.wav ");

		// US sensor 
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE) < 0) {
			printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE)\n");
		}
		else {
			if (write_reg_i2c(file_i2c, 0x02, 0x8C) == -1) {
				printf("\n\nwrite bad...\n");
			}
			else {
				if(pthread_create(&p_thread[thread_nb++], NULL, thread_i2c_ultrasound, (void *)NULL) != 0)
					fprintf(stderr, "Error creating the thread (US)");
			}
		}

	}


	/* Battery check */
	file_adc = open("/dev/twl4030-madc", O_RDWR | O_NONBLOCK);
	system("aplay /var/sons_robot/adc.wav ");
	if (file_adc == -1)
	{
		system("aplay /var/sons_robot/nok.wav ");
		printf("could not open ADC\n");
	}
	else {
		system("aplay /var/sons_robot/ok.wav ");
	}

#ifdef __cplusplus
	par = (twl4030_madc_user_parms *) malloc(sizeof(struct twl4030_madc_user_parms));
#else
	par = malloc(sizeof(struct twl4030_madc_user_parms));
#endif

	battery_voltage = read_adc(file_adc, par, 0);
	if( battery_voltage < 1.7  ) {
		system("aplay /var/sons_robot/low_battery.wav ");
		fprintf(stdout, "LOW BATTERY.\n");
	}
	else {
		system("aplay /var/sons_robot/battery_ok.wav ");
		fprintf(stdout, "LOW BATTERY.\n");
	}

	/* Create the thread for battery monitoring */
	if(pthread_create(&p_thread[thread_nb++], NULL, thread_adc_battery, (void *)NULL) != 0)
		fprintf(stderr, "Error creating the thread");

	/* Create the thread for input values */
#ifdef NEW_INPUT_PROTOCOL
	if(pthread_create(&p_thread[thread_nb++], NULL, getInput, (void *)NULL) != 0)
		fprintf(stderr, "Error creating the thread");
#endif

	/*
	   file_gpio146 = open("/sys/class/gpio/gpio146/value", O_RDWR | O_NONBLOCK);
	   printf("Opening GPIO 146 ... ");
	   if (file_gpio146 == -1)
	   {
	   printf("could not open GPIO146\n");
	   }
	   else {
	   printf("OK \n");
	   }
	   read(file_gpio146, gpio146_input, 1);
	 */

}


int main(void)
{
	// Declare your local variables here
	int wait_counter = 0;


	open_interfaces();

	// Global enable interrupts



	//====================================================================
	//[INIT]

	horizontal_turret = 1500;
	vertical_turret = 1500; // 2150

	//Gait
	GaitType = 8;
	BalanceMode = 0;
	LegLiftHeight = 100;
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

	system("aplay /var/sons/bienvenu a la maison.wav &");


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

#ifndef NEW_INPUT_PROTOCOL
		getInput();
#endif  

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

		if (BalanceMode>0) {
			BalCalcOneLeg(-RFPosX+BodyPosX+RFGaitPosX, RFPosZ+BodyPosZ+RFGaitPosZ,RFGaitPosY, RFOffsetX, RFOffsetZ);
			BalCalcOneLeg( -RMPosX+BodyPosX+RMGaitPosX, RMPosZ+BodyPosZ+RMGaitPosZ,RMGaitPosY, RMOffsetX, RMOffsetZ);
			BalCalcOneLeg( -RRPosX+BodyPosX+RRGaitPosX, RRPosZ+BodyPosZ+RRGaitPosZ,RRGaitPosY, RROffsetX, RROffsetZ);
			BalCalcOneLeg( LFPosX-BodyPosX+LFGaitPosX, LFPosZ+BodyPosZ+LFGaitPosZ,LFGaitPosY, LFOffsetX, LFOffsetZ);
			BalCalcOneLeg( LMPosX-BodyPosX+LMGaitPosX, LMPosZ+BodyPosZ+LMGaitPosZ,LMGaitPosY, LMOffsetX, LMOffsetZ);
			BalCalcOneLeg( LRPosX-BodyPosX+LRGaitPosX, LRPosZ+BodyPosZ+LRGaitPosZ,LRGaitPosY, LROffsetX, LROffsetZ);
			BalanceBody();
		}


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
		for(wait_counter = 0; wait_counter < 10; wait_counter++) {
			usleep(ActualGaitSpeed*100);
			if ( us_sensor_distance < 15 && TravelLengthZ < 0) {
				TravelLengthZ = 0;
			}
		}
		if(sleeping == 0){
			ServoDriver();
		}
		else {
			TravelRotationY=0;
			TravelLengthZ = 0;
			TravelLengthX = 0;
			BodyPosYint = 0;
		}

	};

	return 0;
}
