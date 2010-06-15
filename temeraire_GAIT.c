
#include "temeraire_GAIT.h"

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


						if(GaitLegNr == LRGaitLegNr) { // Left Rear
							leg_on_floor = 0;
							file_gpio146 = open("/sys/class/gpio/gpio146/value", O_RDWR | O_NONBLOCK);
							read(file_gpio146, gpio146_input, 1);
							close(file_gpio146);
							printf("gpio146_input[0] = %d \n",gpio146_input[0]);
							if(gpio146_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}
						if(GaitLegNr == RRGaitLegNr) { // Right Rear
							leg_on_floor = 0;
							file_gpio147 = open("/sys/class/gpio/gpio147/value", O_RDWR | O_NONBLOCK);
							read(file_gpio147, gpio147_input, 1);
							close(file_gpio147);
							printf("gpio147_input[0] = %d \n",gpio147_input[0]);
							if(gpio147_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}
						if(GaitLegNr == LMGaitLegNr) { // Left Middle
							leg_on_floor = 0;
							file_gpio114 = open("/sys/class/gpio/gpio114/value", O_RDWR | O_NONBLOCK);
							read(file_gpio114, gpio114_input, 1);
							close(file_gpio114);
							printf("gpio114_input[0] = %d \n",gpio114_input[0]);
							if(gpio114_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}
						if(GaitLegNr == RMGaitLegNr) { // Right Middle
							leg_on_floor = 0;
							file_gpio186 = open("/sys/class/gpio/gpio186/value", O_RDWR | O_NONBLOCK);
							read(file_gpio186, gpio186_input, 1);
							close(file_gpio186);
							printf("gpio186_input[0] = %d \n",gpio186_input[0]);
							if(gpio186_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}
						if(GaitLegNr == LFGaitLegNr) { // Left Front
							leg_on_floor = 0;
							file_gpio144 = open("/sys/class/gpio/gpio144/value", O_RDWR | O_NONBLOCK);
							read(file_gpio144, gpio144_input, 1);
							close(file_gpio144);
							printf("gpio144_input[0] = %d \n",gpio144_input[0]);
							if(gpio144_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}
						if(GaitLegNr == RFGaitLegNr) { // Right Front
							leg_on_floor = 0;
							file_gpio145 = open("/sys/class/gpio/gpio145/value", O_RDWR | O_NONBLOCK);
							read(file_gpio145, gpio145_input, 1);
							close(file_gpio145);
							printf("gpio145_input[0] = %d \n",gpio145_input[0]);
							if(gpio145_input[0] == 48) { // leg on the floor
								leg_on_floor = 1;
								printf("GaitPosY = %d \n",GaitPosY);
								GaitPosY = GaitPosYY - 3;
								// remettre la vitesse normale
								ActualGaitSpeed = NomGaitSpeed;
							}
							else { // must down the leg
								GaitPosY = GaitPosYY + down_leg_step;
								// Mettre une vitesse rapide pour descendre la patte
								ActualGaitSpeed = DOWN_SENSOR_SPEED;
							}
						}

						GaitPosX = TravelLengthX/2;
						GaitPosZ = TravelLengthZ/2;
						GaitRotY = TravelRotationY/2;
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



