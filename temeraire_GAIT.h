#include "temeraire.h"
#include "temeraire_var.h"
#include "temeraire_UTILS.h"

#ifndef TEMERAIRE_GAIT_H
#define TEMERAIRE_GAIT_H

void GaitSelect(void);
void Gait(char GaitLegNr, signed int GaitPosXX, signed int GaitPosYY, signed int GaitPosZZ, signed int GaitRotYY);
void GaitSeq(void);

#endif
