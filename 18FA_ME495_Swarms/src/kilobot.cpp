#pragma once
#include "kilolib.h"
#include <iostream>
using namespace std;

#define PI 3.14159265358979324

//Weighs on each type of motion -- namely on random
#define W_FWD 1.0
#define W_RAND 0.05
#define W_BUMP 1.0
#define KREP 0.2

//Number of ticks for each type of motion
#define ITSPERDEG 20 		//Number of iterations for one degree (in rads)
#define ITSPERSTP 3 		//Number of iterations to move one unit forward
//"^size of the unit vector

//Variables to assign robot spacing. Found after hours of work
#define ROBOTSIZE 40.0					//size of robot, built in offset to radius
#define BOUNDRY ROBOTSIZE + 25	//space between robots; robot size + arbt. num.
#define SETTLINGTIME 5+1				//Number of iterations + 1 to listen to messages

//Cap on Repulsion Vector
#define MAXREPMAG 50

class mykilobot : public kilobot
{
	//Vars from templates
	unsigned char distance;
	message_t out_message;
	int rxed=0;
	int msrx=0;

	//Colors
	int R = 1;
	int G = 0;
	int B = 0;

	//Iterable/one-shot vars
	bool first = true;
	int iter = 0;
	int state = 0;

	//Variables for intermediate phases and storage
	//idx: 0 -> phototaxis, 1 -> random, 2 -> bumping
	double x[3] = {0.0, 0.0, 0.0};
	double y[3] = {0.0, 0.0, 0.0};
	// Weights for the vectors. Weights played with in setup
	double w [3] = {0.0, 0.0, 0.0};

	// Master Vector
	double cappaX = 0.0;
	double cappaY = 0.0;
	double mag = 0.0;
	double th = 0.0;
	int mag_cnt = 0;
	int ang_cnt = 0;

	//Random Motion Vector
	double rand_num = 0.0;

	//Spacing Variable
	int personal_space = 0;

	//repulsion vars
	double x_rep = 0.0;
	double y_rep = 0.0;
	double rep_mag = 0.0;
	double theta_rep = 0.0;

	// Functions to print angles etc
	double dToR(double d){
		return d*(PI/180);}

	double rToD(double r){
		return r*(180/PI);
	}

	//main loop
	void loop(){
		//On first loop, set colors. Doesn't work in setup for some reason
		//appears that setup runs before all the robots are set in the environment.
		if(first){
			if(id == 0){
				set_color(RGB(0,0,3));
			}
			else if(id == 1){
				set_color(RGB(0,3,0));
			}
			else if(id == 2){
					set_color(RGB(3,0,0));
				}
			else{
				//debug case
				set_color(RGB(3,3,3));
			}
			first = false;
			//compute the personal space val (radius for repel)
			// rad*ring num + some extra to clearly define the rings
			personal_space = (id+1) * BOUNDRY + id*(ROBOTSIZE/4+1);

		}

		//State machine for the movements
		switch (state){
			case 0:
				// count up iter to some value, set to zero, switch.
				// Distance vector is updated in message rx
				if(iter < SETTLINGTIME){
					if(iter == 0){
						//Zero out the x and y vecs before we do anything else...
						x_rep = 0.0;
						y_rep = 0.0;
					}
					iter++;
				}
				else{
					iter = 0;
					state++;
				}
				break;

			case 1:
				//Compute phototaxis stuff
				//unit vector, only cos/sin values needed
				x[0] = cos(angle_to_light);
				y[0] = sin(angle_to_light);
				state++;
				break;

			case 2:
				//Generate a random unit vector for motion
				rand_num = (((double) (rand() % 101))/100.0)*2*PI;
				x[1] = cos(rand_num);
				y[1] = sin(rand_num);
				state++;
				break;

			//Repulsion and vector component addition
			case 3:
				//Check to make sure the repulsion vector isn't too big
				rep_mag = sqrt((pow(x_rep,2)+pow(y_rep,2)));
				//if it is greater..
				if(rep_mag > MAXREPMAG){
					//Grab the direction of the oversized vector
					theta_rep = atan2(y_rep,x_rep);
					//split the max mag into the two directions.
					x_rep = MAXREPMAG*cos(theta_rep);
					y_rep = MAXREPMAG*sin(theta_rep);
				}

						//Store the repulsion vectors and sum up the vectors
				x[2] = x_rep;
				y[2] = y_rep;
				//X and Y computation
				cappaX = 0.0;
				cappaY = 0.0;
				for(int i = 0; i<3; i ++){
					//X
					cappaX += x[i]*w[i];
					//Y
					cappaY += y[i]*w[i];
				}
				state++;
				break;

			//Rotational Motion
			case 4:
				//Find theta and rotate properly.
				if(iter == 0){
					//angle computation
					th = atan2(cappaY,cappaX);
					//iterations + 2 computation and make sure it's a positive number
					if(th >= 0){
						ang_cnt = int(th*ITSPERDEG)+2;
					}
					else{
						ang_cnt = int(-1.0*th*ITSPERDEG)+2;
					}
					//Filter the noise out ~within > 10 deg doesn't do very much.
					if(ang_cnt < 0 || (th > -0.05  && th < 0.05)){
						ang_cnt = 0;
					}
					//Spin up motors
					spinup_motors();
					iter ++;
				}
				else if(iter < ang_cnt){
					//CW is +
					if(th > 0){
						//rotate CW, incement iter
						spinup_motors();
						set_motors(50,0);
						iter++;
					}
					//CCW is -
					else if(th < 0) {
						// rotate CCW, increment iter
						spinup_motors();
						set_motors(50,0);
						iter++;
					}
					else{
						//do no rotation for the zero case
						iter ++;
					}
				}
				else{
					//next state and zero the iter and stop the motors
					spinup_motors();
					set_motors(0,0);
					state++;
					iter = 0;
				}
				break;

			case 5:
			//Linear Motion
				if(iter == 0){
					//find the magnitude
					mag = sqrt(pow(cappaX,2)+pow(cappaY,2));
					//Find the counts for the magnitude +1
					mag_cnt = int((mag+0.5)*ITSPERSTP)+1;
					//Move for that number of counts
					iter++;
				}

				//move forward
				else if (iter > 0 && iter < mag_cnt){
					spinup_motors();
					set_motors(50,50);
					iter++;
				}
				else{
					//next state and zero the iter and stop the motors
					spinup_motors();
					set_motors(0,0);
					state = 0;
					iter = 0;
				}
				break;
		}
	}

	//executed once at start
	void setup(){
		//Initialize the motors for first time.
		spinup_motors();
		//setup the weights
		w[0] = W_FWD;
		w[1] = W_RAND;
		w[2] = W_BUMP;
		out_message.type = NORMAL;
		out_message.data[0] = 10; //nonsense data.
		// we don't care about the data, only the implication of GETTING data
	}

	//executed on successfull message send ; from example code/proj 0
	void message_tx_success(){
		msrx=1;
	}

	//sends message at fixed rate ; from example code/proj 0
	message_t *message_tx()
	{
		static int count = rand();
		count--;
		if (!(count % 50))
		{
			return &out_message;
		}
		return NULL;
	}

	//receives message ; from example code/proj 0
	// Modified for this assignment
	void message_rx(message_t *message, distance_measurement_t *distance_measurement,float t)
	{
		distance = estimate_distance(distance_measurement);
		theta=t;
		//If in the settling time window
		if(state == 0){
			//if the distance measure is less than 2*radius
			if(distance <= 2*personal_space){
				x_rep += -KREP*(personal_space - distance)*sin(theta);
				y_rep += -KREP*(personal_space - distance)*cos(theta);
			}
		}
		rxed = 1;
	}
};
