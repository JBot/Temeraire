
#include "temeraire.h"
#include "temeraire_var.h"
#include "temeraire_THREADS.h"
#include "temeraire_GAIT.h"
#include "temeraire_IK.h"
#include "temeraire_UTILS.h"

/* Functions */
// I/O Functions
void open_interfaces(void);

void firstposition(void);

void firstposition(void) {


	if(starting == 0) {

		/* VRAIE POSITION */ 
		RFPosX = 60;      //Start positions of the Right Front leg
		RFPosY = 25;
		RFPosZ = -71;

		RMPosX = 90;   //Start positions of the Right Middle leg
		RMPosY = 25;
		RMPosZ = -10;	

		RRPosX = 73;    //Start positions of the Right Rear leg
		RRPosY = 25;
		RRPosZ = 51;

		LFPosX = 60;      //Start positions of the Left Front leg
		LFPosY = 25;
		LFPosZ = -71;

		LMPosX = 90;   //Start positions of the Left Middle leg
		LMPosY = 25;
		LMPosZ = -10;

		LRPosX = 73;      //Start positions of the Left Rear leg
		LRPosY = 25;
		LRPosZ = 51;


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

		//adapt_heigth();

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
