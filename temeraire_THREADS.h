
#include "temeraire.h"
#include "temeraire_var.h"
#include "temeraire_GAIT.h"
#include "temeraire_IK.h"
#include "temeraire_UTILS.h"

#ifndef TEMERAIRE_THREADS_H
#define TEMERAIRE_THREADS_H

#ifdef NEW_INPUT_PROTOCOL
void *getInput(void *args);
#else 
void getInput(void);
#endif

// Threads
void *thread_adc_battery(void *arg);
void *thread_i2c_ultrasound(void *args);

#endif
