#include "temeraire.h"
#include "temeraire_var.h"
#include "temeraire_UTILS.h"

#ifndef TEMERAIRE_AI_H
#define TEMERAIRE_AI_H

/* Stuctures */
struct front_map {
        int left_LEFT;
	int left_FRONT;
	int left_RIGHT;

	int front_LEFT;
        int front_FRONT;
        int front_RIGHT;

	int right_LEFT;
        int right_FRONT;
        int right_RIGHT;
};

/* Variables */
extern struct front_map my_front_map;

/* Functions */
void look_around(void);
char check_sensors(void);
void choose_direction_to_avoid_item(int prefered_direction);
void find_where_to_turn(int prefered_direction);
void process_AI(void);

#endif

