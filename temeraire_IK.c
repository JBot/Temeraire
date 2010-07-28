
#include "temeraire_IK.h"

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

void setRotPoint(signed int z) {

	RotPoint = z;

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

void do_IKs(void) {

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


}
