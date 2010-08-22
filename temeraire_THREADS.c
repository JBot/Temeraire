
#include "temeraire_THREADS.h"

void *thread_i2c_ultrasound(void *arg) {
	uint16_t result, result2;
	fprintf(stdout, "Thread: %d (Ultrasound sensor) running.\n", (int)2);
	usleep(70000);
	while(1) {
/*		// Start sensing on the left
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_LEFT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_LEFT)\n");
                }
                else {
		if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
			printf("\n\nwrite bad...\n");
		}
		// Start sensing on the right
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_RIGHT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_RIGHT)\n");
                }
                else {
                if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
                        printf("\n\nwrite bad...\n");
                }
		// Wait end of sensing
		usleep(40000);
		// Start sensing on the front
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_FRONT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_RIGHT)\n");
                }
                else {
                if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
                        printf("\n\nwrite bad...\n");
                }
		usleep(1000);
		// Read left value
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_LEFT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_LEFT)\n");
                }
                else {
                
		result = (uint16_t)read_reg_i2c(file_i2c, 0x02, 1);
                result2 = (uint16_t)read_reg_i2c(file_i2c, 0x03, 1);

		us_sensor_distance_left = ((result & 0x00FF) << 8) + (result2 &  0x00FF);

		}
		usleep(1000);
		// Read right value
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_RIGHT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_RIGHT)\n");
                }
                else {
                
                result = (uint16_t)read_reg_i2c(file_i2c, 0x02, 1);
                result2 = (uint16_t)read_reg_i2c(file_i2c, 0x03, 1);

                us_sensor_distance_right = ((result & 0x00FF) << 8) + (result2 &  0x00FF);

                }
		// Wait end of front sensing
		usleep(40000);
		// Read front value (and light value)
		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_FRONT) < 0) {
                        printf("ERROR : ioctl(I2C_SLAVE, US_DEVICE_FRONT)\n");
                }
                else {

                result = (uint16_t)read_reg_i2c(file_i2c, 0x02, 1);
                result2 = (uint16_t)read_reg_i2c(file_i2c, 0x03, 1);

                us_sensor_distance_front = ((result & 0x00FF) << 8) + (result2 &  0x00FF);
		us_sensor_light = (uint16_t)read_reg_i2c(file_i2c, 0x01, 1);
                }
		usleep(1000);
*/
		/*printf("1 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		  result = (uint16_t)read_reg_i2c(file_i2c, 0x04, 1);
		  result2 = (uint16_t)read_reg_i2c(file_i2c, 0x05, 1);
		  printf("2 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		  result = (uint16_t)read_reg_i2c(file_i2c, 0x06, 1);
		  result2 = (uint16_t)read_reg_i2c(file_i2c, 0x07, 1);
		  printf("3 : IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		 */

		//printf("IO2 : %i IO3 : %i\n", result & 0x00FF, result2 &  0x00FF);
		//printf("us_sensor : %i \n", us_sensor_distance);








		if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_FRONT) < 0) {
			perror("ioctl(I2C_SLAVE)");
		}

//		printf("Running some tests on device 0xE0...\n");
		if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
			printf("\n\nwrite bad...\n");
		usleep(100000);

		result = read_reg_i2c(file_i2c, 2, 1);
		result2 = read_reg_i2c(file_i2c, 3, 1); 
		if (result == -1)
			printf("\n\nread bad...\n"); 
		
		us_sensor_distance_front = ((result & 0x00FF) << 8) + (result2 &  0x00FF);
		//printf("1 -- IO2 : %i IO3 : %i\n", result & 0xff, result2 &  0xff);


                if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_LEFT) < 0) {
                        perror("ioctl(I2C_SLAVE)");
                }

//                printf("Running some tests on device 0xE2...\n");
                if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
                        printf("\n\nwrite bad...\n");
                usleep(100000);

                result = read_reg_i2c(file_i2c, 2, 1);
                result2 = read_reg_i2c(file_i2c, 3, 1);
                if (result == -1)
                        printf("\n\nread bad...\n");

		us_sensor_distance_left = ((result & 0x00FF) << 8) + (result2 &  0x00FF);
                //printf("2 -- IO2 : %i IO3 : %i\n", result & 0xff, result2 &  0xff);


                if (ioctl(file_i2c, I2C_SLAVE, US_DEVICE_RIGHT) < 0) {
                        perror("ioctl(I2C_SLAVE)");
                }

//                printf("Running some tests on device 0xE4...\n");
                if (write_reg_i2c(file_i2c, 0, 0x51) == -1)
                        printf("\n\nwrite bad...\n");
                usleep(100000);

                result = read_reg_i2c(file_i2c, 2, 1);
                result2 = read_reg_i2c(file_i2c, 3, 1);
                if (result == -1)
                        printf("\n\nread bad...\n");

		us_sensor_distance_right = ((result & 0x00FF) << 8) + (result2 &  0x00FF);
                //printf("3 -- IO2 : %i IO3 : %i\n", result & 0xff, result2 &  0xff);







	}
	fprintf(stdout, "Thread: %d done.\n", (int)2);

	pthread_exit(0);
}




void *thread_adc_battery(void *arg) {
	float battery_voltage = 0.0;
	fprintf(stdout, "Thread: %d running.\n", (int)1);

	while(1) {

		sleep((int)60);

		battery_voltage = read_adc(file_adc, par, 0);
		if( battery_voltage < 1.7  ) {
			//system("aplay /var/sons_robot/low_battery.wav &");
			//fprintf(stdout, "LOW BATTERY.\n");
		}

	}
	fprintf(stdout, "Thread: %d done.\n", (int)1);

	pthread_exit(0);
}




#ifdef NEW_INPUT_PROTOCOL
void *getInput(void *args) {

	char my_input[50];
	char read_flag;
	char Serout[260]={0};
	int i=0;
	char speak_buff[300]={0};

	printf("Thread Input running.\n");
	while(1) {
		usleep(8000);

		/* New test of comm protocol */

		if( wait_command_flag == 1) {
			/* Waiting command */

			read_flag = read(ser_fd_modem, my_input, 1);

			if( (read_flag != 0) && (my_input[0] != 0) ) {

				if(read_flag == -1){
					printf("Reading error.");
				}
				else {
					modem_command = my_input[0];
					wait_command_flag = 0;
					temp_input = 0;
					printf("Command received \n");
				}
			}
		}


		if( wait_command_flag == 0) {
			/* Waiting data */

			read_flag = read(ser_fd_modem, my_input, 1);

			if( (read_flag != 0) && (my_input[0] != 0) ) {

				if(read_flag == -1){
					printf("Reading error.");
				}
				else {

					if( my_input[0] == '\n' ) { // End of datas

						switch(modem_command) {

							case 'A' : // Balance mode
								BalanceMode = temp_input;
								break;
							case 'a' : // LEG SENSOR mode
								leg_sensor_ON = (char) abs(temp_input);
								if(leg_sensor_ON == 0)
									ActualGaitSpeed = NomGaitSpeed;
								printf("Leg sensor ON = %d \n",leg_sensor_ON);
								break;
							case 'B' : // BodyPosX
								BodyPosXint = temp_input;
								break;
							case 'b' : 
								BodyPosXint = temp_input;
								break;
							case 'C' : // BodyPosY
								BodyPosYint = temp_input;
								break;
							case 'c' : 
								BodyPosYint = temp_input;
								break;
							case 'D' : // BodyPosZ
								BodyPosZint = temp_input;
								break;
							case 'd' : 
								BodyPosZint = temp_input;
								break;
							case 'E' : // BodyRotX
								BodyRotX = temp_input;
								break;
							case 'e' : 
								BodyRotX = temp_input;
								break;
							case 'F' : // BodyRotY
								BodyRotY = temp_input;
								break;
							case 'f' : 
								BodyRotY = temp_input;
								break;
							case 'G' : // BodyRotZ
								BodyRotZ = temp_input;
								break;
							case 'g' : 
								BodyRotZ = temp_input;
								break;
							case 'H' : // GaitType
								GaitType = temp_input;
								leg_on_floor = 1;
								GaitSelect();
								leg_on_floor = 1;
								break;
							case 'h' : 

								break;
							case 'I' : // GaitSpeed
								read_IMU(); // test
								if( leg_sensor_ON == 1 ) 
									NomGaitSpeed = temp_input;
								else 
									ActualGaitSpeed = temp_input;

								break;
							case 'i' : 
								NomGaitSpeed = temp_input;
								break;
							case 'J' : // Mandibles
								Mandible = temp_input;
								break;
							case 'j' :
								Mandible = temp_input;
								break;
							case 'O' : // ON / OFF
								sleeping = 0;
								starting = 0;
								leg_on_floor = 1;
								system("aplay /var/sons_r2d2/r2d25.wav &");
								break;
							case 'o' : // OFF
								FreeServos();
								TravelRotationY=0;
								TravelLengthZ = 0;
								TravelLengthX = 0;
								sleeping = 1;
								BodyPosYint = 0;
								leg_on_floor = 1;
								system("aplay /var/sons_r2d2/r2d28.wav &");
								break;
							case 'R' : // RotationPoint
								RotPoint = temp_input;
								break;
							case 'r' : 
								RotPoint = temp_input;
								break;
							case 'S' : // Speak
								Serout[i++]='\0';
								//printf("%s",Serout);
								sprintf(speak_buff,"espeak -a 200 -v fr \"%s\" &",Serout);
								system(speak_buff);
								i = 0;
								break;
							case 's' :
								Serout[i++]='\0';
								//printf("%s",Serout);
								sprintf(speak_buff,"espeak -a 200 -v en \"%s\" &",Serout);
								system(speak_buff);
								i = 0;
								break;
							case 'T' : // TravelLengthX
								TravelLengthX = temp_input;
								break;
							case 't' : 
								TravelLengthX = temp_input;
								break;
							case 'U' : //TravelRotY
								//printf("Positif \n");
								TravelRotationY = temp_input;
								break;
							case 'u' : 
								//printf("Negatif \n");
								TravelRotationY = temp_input;
								break;
							case 'V' : // TravelLengthZ
								TravelLengthZ = temp_input;
								break;
							case 'v' : 
								TravelLengthZ = temp_input;
								break;
							case 'Z' : // Special commands

								break;
							case 'z' : 

								break;


							default : break;			


						}
						wait_command_flag = 1;

						temp_input = 0;
					}
					else { // New data

						if( (modem_command == 'S') || (modem_command == 's')) { // Speak command, so don't modify characters

							Serout[i++]=my_input[0];

						}
						else {
							if( modem_command < 92 )
								temp_input = (temp_input * 10) + (my_input[0] - 48);
							else 
								temp_input = (temp_input * 10) - (my_input[0] - 48);
						}
						//printf(" temp_input = %d \n", temp_input);
					}
				}
			}
		}
	}
	printf("Thread Input ended.\n");
}

#else

void getInput(void) {

	char my_input[50];
	char read_flag;
	char Serout[260]={0};

	read_flag = read(ser_fd_modem, my_input, 1);

	if( (read_flag != 0) || (my_input[0] != 0) ) {

		if(read_flag == -1){
			printf("Reading error.");
		}
		else {

			switch(my_input[0]) {

				case 'v' :
					RotPoint = -82; //(avant du robot)
					break; 
				case 'b' :
					RotPoint = 0; //(milieu du robot)
					break;        
				case 'n' :
					RotPoint = 82; //(arriere du robot)
					break;
				case 'B' :
					if(BalanceMode == 0){
						BalanceMode = 1;
						printf("Balance Mode ON  \n");
					}
					else {
						BalanceMode = 0;
						printf("Balance Mode OFF \n");
					}
					break;
				case 'w' :
					BodyPosZint += 5;
					break;        
				case 'x' :
					BodyPosZint -= 5;
					break;        
				case 'z' :
					BodyRotX += 5;
					break;        
				case 's' :
					BodyRotX -= 5;
					break;
				case 'q' :
					BodyPosXint += 5;
					break;
				case 'd' :
					BodyPosXint -= 5;
					break;        
				case '1' :
					TravelRotationY += 10;
					break;        
				case '2' :
					TravelLengthZ += 10;
					break;        
				case '3' :
					TravelRotationY -= 10;
					break;        
				case '4' :
					TravelRotationY += 10;
					break;        
				case '5' :
					FreeServos();
					TravelRotationY=0;
					TravelLengthZ = 0;
					TravelLengthX = 0;
					sleeping = 1;
					BodyPosYint = 0;
					system("aplay /var/sons_r2d2/r2d28.wav &");
					break;        
				case '6' :
					TravelRotationY -= 10;
					break;        
				case '7' :
					TravelLengthX += 10;
					break;        
				case '8' :
					TravelLengthZ -= 10;
					break;        
				case '9' :
					TravelLengthX -= 10;
					break;        
				case '0' :
					TravelRotationY=0;
					TravelLengthZ = 0;
					TravelLengthX = 0;
					break;        
				case '+' :
					NomGaitSpeed = NomGaitSpeed + 20;
					break;                
				case '-' :
					NomGaitSpeed = NomGaitSpeed - 20;
					break;                
				case '/' :
					if( GaitType == 0)
						GaitType = 8;
					else      
						GaitType--;
					GaitSelect();
					break;
				case '*' :
					if( GaitType == 8)
						GaitType = 0;
					else
						GaitType++;
					GaitSelect();
					break;
				case 'p' :
					sleeping = 0;
					starting = 0;
					system("aplay /var/sons_r2d2/r2d25.wav &");
					break;
				case 'm' :
					sprintf(Serout, "%s US SENSOR : %d ", Serout, us_sensor_distance);
					sprintf(Serout, "%s LIGHT SENSOR : %d \n\r", Serout, us_sensor_light);
					break;

				default : break;
			}

			printf("Char received = %x \n",my_input[0]);


			sprintf(Serout, "%s %d %d %d", Serout, TravelLengthX, TravelRotationY, TravelLengthZ);
			sprintf(Serout, "%s %d %d %d", Serout, BodyPosXint, BodyPosYint, BodyPosZint);
			sprintf(Serout, "%s %d %d %d", Serout, BodyRotX, BodyRotY, BodyRotZ);
			sprintf(Serout, "%s %d %d %d", Serout, GaitType, NomGaitSpeed, RotPoint);/*
												    sprintf(Serout, "%s %d %d %d", Serout, , , );
												    sprintf(Serout, "%s %d %d %d", Serout, , , );
												  */
			sprintf(Serout, "%s \n\r", Serout);
			// write to serial if connected
			if ( ser_fd_modem )
				write(ser_fd_modem, &Serout, sizeof(Serout));



		}

	}		




}
#endif

void read_IMU(void) {

	char my_input[50];
        char read_flag;
        char Serout[260]={0};
        int i=0;
	int16_t roll, pitch; 
	float roll_f, pitch_f;


	// Requesting DATA
	//printf("BEFORE WRITE \n");
	sprintf(Serout, "snp%c%c%c%c", (char)0x01, (char)0x00, (char)0x01, (char)0x52);
	//printf("BEFORE WRITE \n");
        // write to serial if connected
        if ( ser_fd_imu )
                write(ser_fd_imu, &Serout, sizeof(Serout));
	//printf("AFTER WRITE \n");
	usleep(50000);
	//printf("AFTER SLEEP \n");

	// 13 datas a récuperer

	read_flag = read(ser_fd_imu, my_input, 13);
	//printf("AFTER READ \n");
	if( (read_flag != 0) && (my_input[0] != 0) ) {

		if(read_flag == -1){
			printf("!!!! IMU Reading error. !!!!");
		}
		else {
			//modem_command = my_input[0];
			//printf("!!!! IMU READ OK !!!!\n");

			roll = (((int8_t) my_input[7]) << 8) + my_input[8];
			roll_f = roll * 0.0109863;
			pitch = (((int8_t) my_input[9]) << 8) + my_input[10];
			pitch_f = pitch * 0.0109863;

			global_roll = roll_f;
			global_pitch = pitch_f;

			//printf("DATAS : %c%c%c %x %x %x%x %x%x %x%x %x%x \n", my_input[0], my_input[1], my_input[2], my_input[3], my_input[4], my_input[5], my_input[6], my_input[7], my_input[8], my_input[9], my_input[10], my_input[11], my_input[12] );
			printf("DATAS : %c%c%c %x %x %x%x %f %f %x%x \n", my_input[0], my_input[1], my_input[2], my_input[3], my_input[4], my_input[5], my_input[6], roll_f, pitch_f, my_input[11], my_input[12] );
		}
	}

}
