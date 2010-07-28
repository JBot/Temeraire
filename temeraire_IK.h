
#include "temeraire.h"
#include "temeraire_var.h"
#include "temeraire_UTILS.h"

#ifndef TEMERAIRE_IK_H
#define TEMERAIRE_IK_H

void BodyIK(signed int PosX, signed int PosZ, signed int PosY, signed int BodyOffsetX, signed int BodyOffsetZ, signed int RotationY);
void LegIK(signed int IKFeetPosX, signed int IKFeetPosY, signed int IKFeetPosZ);
void LegIK2(signed long IKFeetPosX, signed long IKFeetPosY, signed long IKFeetPosZ);
void setRotPoint(signed int z);
void doBodyRot(void);
void do_IKs(void);

// Balance functions
void BalCalcOneLeg(double my_PosX, double my_PosZ,double my_PosY,int my_BodyOffsetX, int my_BodyOffsetZ);
void BalanceBody();

#endif

