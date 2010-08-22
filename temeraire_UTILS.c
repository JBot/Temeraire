
#include "temeraire_UTILS.h"


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

	//printf("%s \n",Serout);
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

void adapt_height(void) {

	if((LRGaitPosY > LIMIT_HEIGHT) &&
			(RRGaitPosY > LIMIT_HEIGHT) &&
			(LMGaitPosY > LIMIT_HEIGHT) &&
			(RMGaitPosY > LIMIT_HEIGHT) &&
			(LFGaitPosY > LIMIT_HEIGHT) &&
			(RFGaitPosY > LIMIT_HEIGHT)) {

		LRGaitPosY-=10;
		RRGaitPosY-=10;
		LMGaitPosY-=10;
		RMGaitPosY-=10;
		LFGaitPosY-=10;
		RFGaitPosY-=10;

	printf("!!!!!!!!!!!!!!!! \n");
        printf("Height adapted ! \n");
        printf("!!!!!!!!!!!!!!!! \n");

	
	}
}
#define ADAPT_DISTANCE 3
void adapt_pitch_roll(void) {


	if(global_pitch > 7.0) {
                LRGaitPosY_adapt-=ADAPT_DISTANCE;
                RRGaitPosY_adapt-=ADAPT_DISTANCE;

		LFGaitPosY_adapt+=ADAPT_DISTANCE;
                RFGaitPosY_adapt+=ADAPT_DISTANCE;
	}

	if(global_pitch < 4.0) {
                LRGaitPosY_adapt+=ADAPT_DISTANCE;
                RRGaitPosY_adapt+=ADAPT_DISTANCE;

                LFGaitPosY_adapt-=ADAPT_DISTANCE;
                RFGaitPosY_adapt-=ADAPT_DISTANCE;
        }
	if(global_roll > 9.0) {
                LRGaitPosY_adapt-=ADAPT_DISTANCE;
                LMGaitPosY_adapt-=ADAPT_DISTANCE;
                LFGaitPosY_adapt-=ADAPT_DISTANCE;

                RRGaitPosY_adapt+=ADAPT_DISTANCE;
                RMGaitPosY_adapt+=ADAPT_DISTANCE;
                RFGaitPosY_adapt+=ADAPT_DISTANCE;
        }
        if(global_roll < 6.0) {
                LRGaitPosY_adapt+=ADAPT_DISTANCE;
                LMGaitPosY_adapt+=ADAPT_DISTANCE;
                LFGaitPosY_adapt+=ADAPT_DISTANCE;

                RRGaitPosY_adapt-=ADAPT_DISTANCE;
                RMGaitPosY_adapt-=ADAPT_DISTANCE;
                RFGaitPosY_adapt-=ADAPT_DISTANCE;
        }

}


inline struct timeval timeval_difference( struct timeval* first,
                                          struct timeval* second )
{
  struct timeval to_return;

  if( second->tv_usec > first->tv_usec )
    {
      to_return.tv_usec = ( second->tv_usec - first->tv_usec );
      to_return.tv_sec = ( second->tv_sec - first->tv_sec );
    }
  else
    {
      to_return.tv_usec = ( 1000000 - first->tv_usec ) + second->tv_usec;
      to_return.tv_sec = ( second->tv_sec - first->tv_sec ) - 1;
    }

  return to_return;
}

inline int is_greater_than( struct timeval* first, struct timeval* second )
{
  if( first->tv_sec > second->tv_sec ) return 1;
  if( ( first->tv_sec == second->tv_sec ) && ( first->tv_usec > second->tv_usec ) ) return 1;
  return 0;
}


