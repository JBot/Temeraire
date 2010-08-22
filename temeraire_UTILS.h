
#include "temeraire.h"
#include "temeraire_var.h"

#ifndef TEMERAIRE_UTILS_H
#define TEMERAIRE_UTILS_H

// I/O Functions
int read_reg_i2c(int fh, uint8_t reg, int count);
int write_reg_i2c(int fh, uint8_t reg, uint8_t val);
float read_adc(int file, struct twl4030_madc_user_parms *par, int adc_channel);

void GetSinCos (float AngleDeg);
void GetBoogTan(float BoogTanX, float BoogTanY);
void ServoDriver(void);
void FreeServos();
void adapt_height(void);
void adapt_pitch_roll(void);

inline struct timeval timeval_difference( struct timeval* first, struct timeval* second );
inline int is_greater_than( struct timeval* first, struct timeval* second );

#endif

