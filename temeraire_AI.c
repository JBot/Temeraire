#include "temeraire_AI.h"
#include "temeraire_IK.h"

struct front_map my_front_map = { 250, 250, 250, 250, 250, 250, 250, 250, 250};

int speed_saveZ = 0;

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

	//if( (us_sensor_distance_front < US_FRONT_DETECTION) && (us_sensor_distance_left < US_SIDE_DETECTION) && (us_sensor_distance_right < US_SIDE_DETECTION) ) {
	if( (us_sensor_distance_left < US_SIDE_DETECTION) && (us_sensor_distance_right < US_SIDE_DETECTION) ) {
		// Peut etre enlever le front sensor
		// The robot can't continu in front
		printf("Gauche et droite < limit \n");
		result = 1;
	}
	else if( (us_sensor_distance_front < US_FRONT_DETECTION) && (us_sensor_distance_left > US_SIDE_DETECTION) ) {
		// There is an item in front but robot can go left
		printf("front < limit . Gauche > limit \n");
		result = 2;
	}
	else if ( (us_sensor_distance_front < US_FRONT_DETECTION) && (us_sensor_distance_right > US_SIDE_DETECTION) ) {
		// There is an item in front but robot can go right
		printf("front < limit . droite > limit \n");
		result = 3;
	}
	else if ( (us_sensor_distance_front > US_FRONT_DETECTION) && (us_sensor_distance_left < US_SIDE_DETECTION) ) {
		// There is no item but something on the left, so straff on right
		printf("front > limit . Gauche < limit \n");
		result = 4;
	}
	else if ( (us_sensor_distance_front > US_FRONT_DETECTION) && (us_sensor_distance_right < US_SIDE_DETECTION) ) {
		// There is no item but something on the left, so straff on left
		printf("front > limit . droite < limit \n");
		result = 5;
	}
	else {
		// All is ok on both side
		printf("All is ok \n");
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

	printf("processing AI ... %i \n", temeraire_state.state);

	switch(temeraire_state.state) {

		case NOTHING : 
			printf("State doing nothing \n");
			break;

		case CHECKING_ENV :
			       switch(check_sensors()) {
				       case 0 :
					       // Nothing
						printf("Rien a faire \n");
					       break;
				       case 1 :
					       speed_saveZ = TravelLengthZ;
						TravelLengthZ = 0;
					       look_around();
					       temeraire_state.state = AVOIDING_WALL_FRONT;
						printf("Go to avoiding wall front \n");
						system("espeak -a 200 \"Wall detected in front\" &");
					       //find_where_to_turn(1);
					       //gettimeofday( &timebeforenextcheck, NULL );
					       break;
				       case 2 : // front item > go left
					       speed_saveZ = TravelLengthZ;
						TravelLengthX = - TravelLengthZ;
					       temeraire_state.state = AVOIDING_ITEM_LEFT;
						printf("Goto Avoiding item left \n");
						system("espeak -a 200 \"Object detected in front! Avoiding it by the left\" &");
					       break;
				       case 3 :
						speed_saveZ = TravelLengthZ;
					       TravelLengthX = TravelLengthZ;
					       temeraire_state.state = AVOIDING_ITEM_RIGHT;
						printf("Goto Avoiding item right \n");
					       	system("espeak -a 200 \"Object detected in front! Avoiding it by the right\" &");
						break;
				       case 4 :
						speed_saveZ = TravelLengthZ;
					       TravelLengthX = TravelLengthZ;
						TravelRotationY = 5;
					       temeraire_state.state = AVOIDING_WALL_SIDE;
						printf("Goto avoiding wall side \n");
						system("espeak -a 200 \"Wall detected on my left\" &");
					       break;
				       case 5 :
						speed_saveZ = TravelLengthZ;
					       TravelLengthX = - TravelLengthZ;
						TravelRotationY = -5;
					       temeraire_state.state = AVOIDING_WALL_SIDE;
						printf("Goto avoiding wall side \n");
						system("espeak -a 200 \"Wall detected on my right\" &");
					       break;
				       default :
						printf("Default State ?? \n");
					       break;
			       }
			       break;

		case AVOIDING_ITEM_LEFT :
				switch(check_sensors()) {
                                       case 0 :
                                               // Nothing
						printf("Stop avoiding item left \n");
						system("espeak -a 200 \"Object avoided successfully\" &");
						temeraire_state.state = CHECKING_ENV;
						TravelRotationY = 0;
                                		TravelLengthX = 0;
						TravelLengthZ = speed_saveZ;
                                               break;
                                       case 1 :
                                               TravelLengthZ = 0;
                                               look_around();
                                               temeraire_state.state = AVOIDING_WALL_FRONT;
						printf("Goto avoiding wall front \n");
                                               //find_where_to_turn(1);
                                               //gettimeofday( &timebeforenextcheck, NULL );
                                               break;
                                       case 2 : // front item > go left
						TravelLengthZ = speed_saveZ;
						if((us_sensor_distance_front < US_FRONT_LIMIT))
							TravelLengthZ = 0;
						else
							TravelLengthX += 3;
                                               break;
                                       case 3 :
                                               break;
                                       case 4 :
						TravelLengthZ = speed_saveZ;
                                                if((us_sensor_distance_right < US_SIDE_LIMIT))
                                                        TravelLengthZ = 0;
                                                else
                                                        TravelLengthX += 2;
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
                                               printf("Stop avoiding wall front \n");
                                                system("espeak -a 200 \"Wall avoided successfully\" &");
                                                temeraire_state.state = CHECKING_ENV;
                                                TravelRotationY = 0;
                                                TravelLengthX = 0;
                                                TravelLengthZ = speed_saveZ;
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

		case AVOIDING_ITEM_RIGHT :

                                switch(check_sensors()) {
                                       case 0 :
                                               // Nothing
                                                printf("Stop avoiding item right \n");
						system("espeak -a 200 \"Object avoided successfully\" &");
						temeraire_state.state = CHECKING_ENV;
						TravelRotationY = 0;
                                		TravelLengthX = 0;
						TravelLengthZ = speed_saveZ;
                                               break;
                                       case 1 :
                                               TravelLengthZ = 0;
                                               look_around();
                                               temeraire_state.state = AVOIDING_WALL_FRONT;
                                                printf("Goto avoiding wall front \n");
                                               //find_where_to_turn(1);
                                               //gettimeofday( &timebeforenextcheck, NULL );
                                               break;
                                       case 2 : // front item > go left
                                               break;
                                       case 3 :
						TravelLengthZ = speed_saveZ;
                                                if((us_sensor_distance_front < US_FRONT_LIMIT))
                                                        TravelLengthZ = 0;
                                                else
                                                        TravelLengthX -= 3;
                                               break;
                                       case 4 :
                                               break;
                                       case 5 :
						TravelLengthZ = speed_saveZ;
                                                if((us_sensor_distance_left < US_SIDE_LIMIT))
                                                        TravelLengthZ = 0;
                                                else
                                                        TravelLengthX -= 2;
                                               break;
                                       default :

                                               break;
                               }

                               break;

		case AVOIDING_WALL_SIDE :
                                switch(check_sensors()) {
                                       case 0 :
                                               // Nothing
                                                printf("Stop avoiding wall \n");
                                                system("espeak -a 200 \"Wall avoided successfully\" &");
                                                temeraire_state.state = CHECKING_ENV;
                                                TravelRotationY = 0;
                                                TravelLengthX = 0;
                                                TravelLengthZ = speed_saveZ;
                                               break;
                                       case 1 :
                                               TravelLengthZ = 0;
                                               look_around();
                                               temeraire_state.state = AVOIDING_WALL_FRONT;
                                                printf("Goto avoiding wall front \n");
                                               break;
                                       case 2 : // front item > go left
                                                TravelLengthZ = speed_saveZ;
                                                if((us_sensor_distance_front < US_FRONT_LIMIT))
                                                        TravelLengthZ = 0;
                                                else
                                                        TravelLengthX = - speed_saveZ;
                                               break;
						temeraire_state.state = AVOIDING_ITEM_LEFT;
                                       case 3 :
						TravelLengthZ = speed_saveZ;
                                                if((us_sensor_distance_front < US_FRONT_LIMIT))
                                                        TravelLengthZ = 0;
                                                else
                                                        TravelLengthX = speed_saveZ;
                                               break;
                                                temeraire_state.state = AVOIDING_ITEM_RIGHT;
                                               break;
                                       case 4 :
                                               break;
                                       case 5 :
                                               break;
                                       default :
						break;		
				}
				break;


		default : 
			printf("Default state AI \n");
			break;
	}

}
