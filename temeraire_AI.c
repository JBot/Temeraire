#include "temeraire_AI.h"
#include "temeraire_IK.h"

struct front_map my_front_map = { 250, 250, 250, 250, 250, 250, 250, 250, 250};

/* Function to look around when an obstacle occures in front of the robot. */
void look_around(void) {
	printf("looking around ... \n");
	my_front_map.front_LEFT = us_sensor_distance_left;
	my_front_map.front_FRONT = us_sensor_distance_front;
	my_front_map.front_RIGHT = us_sensor_distance_right;


	// Look on the left
	BodyRotY = - 6;
	do_IKs();
	if(sleeping == 0){
		ServoDriver();
	}

	usleep(1000000);	
	// Save distances
	my_front_map.left_LEFT = us_sensor_distance_left;
	my_front_map.left_FRONT = us_sensor_distance_front;
	my_front_map.left_RIGHT = us_sensor_distance_right;

	// Look on the right
	BodyRotY = 6;
	do_IKs();
	if(sleeping == 0){
		ServoDriver();
	}

	usleep(1000000);
	// Save distances
	my_front_map.right_LEFT = us_sensor_distance_left;
	my_front_map.right_FRONT = us_sensor_distance_front;
	my_front_map.right_RIGHT = us_sensor_distance_right;

	BodyRotY = 0;

}

char check_sensors(void) {
	char result = 0;

	if( (us_sensor_distance_front < US_FRONT_LIMIT) && (us_sensor_distance_left < US_SIDE_LIMIT) && (us_sensor_distance_right < US_SIDE_LIMIT) ) {
		// Peut etre enlever le front sensor
		// The robot can't continu in front
		result = 1;
	}
	else if( (us_sensor_distance_front < US_FRONT_LIMIT) && (us_sensor_distance_left > US_SIDE_LIMIT) ) {
		// There is an item in front but robot can go left
		result = 2;
	}
	else if ( (us_sensor_distance_front < US_FRONT_LIMIT) && (us_sensor_distance_right > US_SIDE_LIMIT) ) {
		// There is an item in front but robot can go right
		result = 3;
	}
	else if ( (us_sensor_distance_front > US_FRONT_LIMIT) && (us_sensor_distance_left < US_SIDE_LIMIT) ) {
		// There is no item but something on the left, so straff on right
		result = 4;
	}
	else if ( (us_sensor_distance_front > US_FRONT_LIMIT) && (us_sensor_distance_right < US_SIDE_LIMIT) ) {
		// There is no item but something on the left, so straff on left
		result = 5;
	}
	else {
		// All is ok on both side
		result = 0;
	}
	return result;

}

void choose_direction_to_avoid_item(int prefered_direction) {
	// prefered direction is the right
	if(prefered_direction) {
		if( (my_front_map.front_RIGHT > US_SIDE_LIMIT ) ) {
			// Go on the right
			TravelLengthX = - TravelLengthZ;
		}
		else if (  (my_front_map.front_LEFT > US_SIDE_LIMIT ) ) {
			// Go on the left
			TravelLengthX = TravelLengthZ;
		}
		else {
			// Can't go anywhere

		}

	}
	else {
		if ( (my_front_map.front_LEFT > US_SIDE_LIMIT ) ) {
			// Go on the left
			TravelLengthX = TravelLengthZ;
		}
		else if( (my_front_map.front_RIGHT > US_SIDE_LIMIT ) ) {
			// Go on the right
			TravelLengthX = - TravelLengthZ;
		}
		else {
			// Can't go anywhere

		}

	}
}

void find_where_to_turn(int prefered_direction) {
	// prefered direction is the right
	if(prefered_direction) {
		if( (my_front_map.right_RIGHT > US_SIDE_LIMIT ) && (my_front_map.front_RIGHT > US_SIDE_LIMIT ) ) {
			// Go on the right
			TravelRotationY = 10;
		}
		else if ( (my_front_map.left_LEFT > US_SIDE_LIMIT ) && (my_front_map.front_LEFT > US_SIDE_LIMIT ) ) {
			// Go on the left
			TravelRotationY = -10;
		}
		else {
			// Can't go anywhere

		}

	}
	else {
		if ( (my_front_map.left_LEFT > US_SIDE_LIMIT ) && (my_front_map.front_LEFT > US_SIDE_LIMIT ) ) {
			// Go on the left
			TravelRotationY = -10;
		}
		else if( (my_front_map.right_RIGHT > US_SIDE_LIMIT ) && (my_front_map.front_RIGHT > US_SIDE_LIMIT ) ) {
			// Go on the right
			TravelRotationY = 10;
		}
		else {
			// Can't go anywhere

		}

	}
}

void process_AI(void) {

	switch(temeraire_state.state) {

		case NOTHING : break;

		case CHECKING_ENV :
				TravelRotationY = 0;
				TravelLengthX = 0;
			       switch(check_sensors()) {
				       case 0 :
					       // Nothing
					       break;
				       case 1 :
					       TravelLengthZ = 0;
					       look_around();
					       temeraire_state.state = AVOIDING_WALL_FRONT;
					       //find_where_to_turn(1);
					       //gettimeofday( &timebeforenextcheck, NULL );
					       break;
				       case 2 : // front item > go left
					       TravelLengthX = TravelLengthZ;
					       temeraire_state.state = AVOIDING_ITEM_LEFT;
					       break;
				       case 3 :
					       TravelLengthX = - TravelLengthZ;
					       temeraire_state.state = AVOIDING_ITEM_RIGHT;
					       break;
				       case 4 :
					       TravelLengthX = TravelLengthZ;
					       temeraire_state.state = AVOIDING_WALL_SIDE;
					       break;
				       case 5 :
					       TravelLengthX = - TravelLengthZ;
					       temeraire_state.state = AVOIDING_WALL_SIDE;
					       break;
				       default :

					       break;
			       }
			       break;

		case AVOIDING_ITEM_LEFT :

				switch(check_sensors()) {
                                       case 0 :
                                               // Nothing
						TravelLengthX = 0;
                                               break;
                                       case 1 :
                                               TravelLengthZ = 0;
                                               look_around();
                                               temeraire_state.state = AVOIDING_WALL_FRONT;
                                               //find_where_to_turn(1);
                                               //gettimeofday( &timebeforenextcheck, NULL );
                                               break;
                                       case 2 : // front item > go left
                                               break;
                                       case 3 :
                                               break;
                                       case 4 :
                                               break;
                                       case 5 :
                                               break;
                                       default :

                                               break;
                               }

			       break;


		case AVOIDING_WALL_FRONT :
				
				switch(check_sensors()) {
                                       case 0 :
                                               // Nothing
                                                TravelRotationY = 10;
                                               break;
                                       case 1 :
                                               TravelLengthZ = 10; // Go back
                                               //look_around();
                                               //find_where_to_turn(1);
                                               //gettimeofday( &timebeforenextcheck, NULL );
                                               break;
                                       case 2 : // front item > go left
						TravelRotationY = 10;
                                               break;
                                       case 3 :
						TravelRotationY = -10;
                                               break;
                                       case 4 :
						TravelRotationY = 10;
                                               break;
                                       case 5 :
						TravelRotationY = -10;
                                               break;
                                       default :

                                               break;
                               }



				break;

		default : break;
	}

}
