#pragma once
#include "kilolib.h"

class mykilobot : public kilobot
{
	unsigned char distance;
	message_t out_message;
	int rxed=0;

	int motion=0;
	long int motion_timer=0;
	int msrx=0;

	//Color Variables
	int R = 0;
	int G = 0;
	int B = 0;

	//Stage timers. Count iterations something has happened
	int iter = 0;
	#define ONE 600
	#define TWO 400
	#define THREE 20000
	#define OPTIONAL 400
	//Define error threashold
	#define ERRORTH 0.01
	#define STEPSIZE 0.5

	//variables to toggle stages
	int GRAD = 1;
	int COORD = 0;
	int MAKEN = 0;
	int NEVERSMOOTH = 0;
	//Toggle smoothing on/off here.
	int SMOOTH = 1;
	int DEFAULT = 1;

	//Max hopcount value - it should never go above 34...
	#define maxHopVal 13

	//For hopcount smoothing stuff.
	double counts1 = 0;
	double counts2 = 0;
	double sumhops1 = 0;
	double sumhops2 = 0;
	double hc1 = 0.0;
	double hc2 = 0.0;

	//temp messages for hop count RX-ing
	int old1 = 0;
	int old2 = 0;

	//Structs for handling data in/out
	struct mydata {
		unsigned int data1;
		unsigned int data2;
	};

	//struct for handling x/y coordinates
	struct coords {
		double x;
		double y;
	};

	//Hop counts struct - Now I know why this struct is useful!
	mydata hopcounts;

	//init x-y coordinates structs
	coords myPos;
	coords seed1Pos;
	coords seed2Pos;

	//for computing trilateration
	double dist_seed_1 = 0.0;
	double dist_seed_2 = 0.0;

	double partialX = 0.0;
	double partialY = 0.0;

	// set errors to be nonsinsically  high
	double totalError = 4000.0;

	//learning rate for gradient descent
	double alpha = 0.1;

	//helper functions
	double distanceToSeed(coords sensor, coords seed){
		//From eq (1)
		return pow((pow((seed.x-sensor.x),2) + pow((seed.y-sensor.y),2)),0.5);
	}

	double findErrorSingleStep(double predict, double calc){
		//From eq (2)
		return pow((calc-predict),2);
	}

	double partialDerivativeSingleStep(double sens_xy, double seed_xy,
	 				double hc, double dist_calc){
						// hc is hop count
						// dist_calc is distance from distanceToSeed fcn
						// sens_xy,seed_xy is one of the two x-y coordinates for the sensor,
						// 	and the seed, respectively.
		return -(2.0*(seed_xy-sens_xy)*(dist_calc - hc))/dist_calc;
		//return (sens_xy-seed_xy)*(1-(dist_calc/hc));
	}

	double deltaCoord(double coord, double alpha){
		//from eq (4)
		return -1.0*alpha*coord;
	}

	//main loop
	void loop()
	{
		//RGB Variables - start them at 0 on every loop
		R = 0;
		G = 0;
		B = 0;
		//Run the gradients -- We compute both gradients in one step.
		if(GRAD){
			//Count if the hop count has stabalized, increment this counter
			if(hopcounts.data1 == out_message.data[0] &&
				 hopcounts.data2 == out_message.data[1] && (id !=1 && id !=32)){
				iter ++;
			}
			//Write the result to the out message
			out_message.data[0] = hopcounts.data1;
			out_message.data[1] = hopcounts.data2;

			//Gradient %2 as noted in class - pretty pictures - comment out if done
			if(hopcounts.data1 % 2 == 0){
				G = 1;
			}
			if(hopcounts.data2 % 2 == 0){
				R = 1;
			}

			// if stabalized OR seed, leave loop.
			if((iter > ONE) || id == 1 || id == 32){
				GRAD = 0;
				//enter next phase
				if(!SMOOTH){
					COORD = 1;
				}
				//reset counter
				iter = 0;
			}
		}

		//Smoothing Option
		else if(!GRAD && SMOOTH){
			//Lazy, so, doing this based on timing... Gut feeling?
			if(iter < OPTIONAL){
				//Grab the current hop counts
				hc1 = double(hopcounts.data1);
				hc2 = double(hopcounts.data2);
				//Exponential moving average method (as suggested in office hours)
				//Add in the old to the new F(n) + gamma*F(n-1) where gamma
				// is the counter)
				hc1 = sumhops1 + hc1;
				hc1 = sumhops1 /(counts1+1);

				hc2 = sumhops2 + hc2;
				hc2 = sumhops2 /(counts2+1);

				//Write the CONSTANT hopcounts so everyone knows your old positions.
				out_message.data[0] = hopcounts.data1;
				out_message.data[1] = hopcounts.data2;
			}
			//Increment and transiton out of phase
			iter++;
			if(iter >= OPTIONAL){
				SMOOTH = 0;
				COORD = 1;
				//As said in the paper, at the end, remove 0.5.
				hc1 = hc1 - 0.5;
				hc2 = hc2 - 0.5;
				NEVERSMOOTH = 1;
				iter = 0;
			}
		}

		//Coord generation via trilateration
		else if(!SMOOTH && COORD){
			// generate initial coordinates
			//Trilateration - done in function calls and not loop-ed
			// as we only have two sensors, no need to loop for i \in [1,2]
			// We assume convergence will happen within the time window.
			if(!NEVERSMOOTH){
				hc1 = float(hopcounts.data1);
				hc2 = float(hopcounts.data2);
			}

			if(iter < TWO && !MAKEN){
				R = 0;
				B = 0;
				G = 0;

			  if(iter == 0){
			  	//Let's try random coordinates and see if we converge?
					myPos.x = 4.0;
					myPos.y = 4.0;
				}
					//First we calculate distance to each sensor
					dist_seed_1 = distanceToSeed(myPos,seed1Pos);
					dist_seed_2 = distanceToSeed(myPos,seed2Pos);
					//then we compute the error for the sensor
					totalError = findErrorSingleStep(dist_seed_1, hc1);
					totalError += findErrorSingleStep(dist_seed_2, hc2);
					//if errors are low enough, break
					if(totalError < ERRORTH){
						MAKEN = 1;
					}
					// Compute gradients for each of the four variables and increment
					// X	coordinate
					partialX = partialDerivativeSingleStep(myPos.x, seed1Pos.x, hc1, dist_seed_1);
					partialX += partialDerivativeSingleStep(myPos.x, seed2Pos.x, hc2, dist_seed_2);
					myPos.x = myPos.x + deltaCoord(partialX, alpha);
					//Y coordinate
					partialY = partialDerivativeSingleStep(myPos.y, seed1Pos.y, hc1, dist_seed_1);
					partialY += partialDerivativeSingleStep(myPos.y, seed2Pos.y, hc2, dist_seed_2);
					myPos.y = myPos.y + deltaCoord(partialY, alpha);

					//increment the counter
					iter++;
				}

				//If we timed out finding coordinates
				else{
					COORD = 0;
					MAKEN = 1;
					iter = 0;
				}
			}

		//Make the N
		else if(MAKEN){
			if(iter < THREE){

				//Smooth-specific coordinates
				if(DEFAULT){
					//DEFAULT is TRUE --> GREAT FOR SMOOTHED.
					//STILL behaves well unsmoothed.
					int WALL = 2.1;
					int K = 3; //X VAR
					int Q = 2; //Y VAR

					if(((myPos.y) < WALL || (myPos.y) > (8-WALL))){// && myPos.x < 5){
						R = 2;
						B = 3;
					}
					else if(((myPos.y) < WALL || (myPos.y) > (8-WALL-0.2)) && myPos.x >= 5){
						R = 2;
						B = 3;
					}
					//Cross Section
					//else
					//Not smooth specific
					else if(((Q*myPos.x + K*myPos.y > 19.5) && (Q*myPos.x + K*myPos.y < 25))
					 && myPos.x >= 0)
					 //S\o to my middle school Alg I teacher for telling me Ax+By=C form
					 //had a use. This here is it!^^
					{
						R = 2;
						B = 3;
					}
					else{
						R = 3;
						G = 3;
						B = 3;
					}
				}
				else{
					//Default is false --> Great for not smoothing
					int WALL = 2.1;
					int Q = 2;
					int K = 3.5;
					//Walls
					if((myPos.y) < WALL || (myPos.y) > (8-WALL)){
						R = 2;
						B = 3;
					}
					//Cross Section
					else if((Q*myPos.x + K*myPos.y > 19) && (Q*myPos.x + K*myPos.y < 30)&&
					myPos.x > 3.8 && myPos.x < 6.8)
					{
						R = 2;
						B = 3;
					}
					else{
						R = 3;
						G = 3;
						B = 3;
					}
				}
			//Increment the iterer
			iter++;
			}
		}

		//Force seeds to be constant for-ev-er
		if((id == 1 || id == 32)){
			R = 0;
			B = 0;
			G = 2;
			//Force seeds to hopcount of 1 for all time
			if(id == 1){
				hopcounts.data1 = 1;
				myPos.x = 0;
				myPos.y = 0;
			}
			else if(id == 32){
				hopcounts.data2 = 1;
				myPos.x = 0;
				myPos.y = 31;
			}
			//Write the result to the out message
			out_message.data[0] = hopcounts.data1;
			out_message.data[1] = hopcounts.data2;
		}

		//Color setting and message sending
		out_message.crc = message_crc(&out_message);
		set_color(RGB(R,G,B));
		}

	//executed once at start
	void setup()
	{
		out_message.type = NORMAL;
		//Set the initial hop counts to the MAX
		hopcounts.data1 = maxHopVal;
		hopcounts.data2 = maxHopVal;
		out_message.data[0] = hopcounts.data1;
		out_message.data[1] = hopcounts.data2;
		//We define the message structure as
		//out_message 0 - hop counts from robot 1
		//out_message 1 - hop counts from robot 32

		//Setup coordinates
		myPos.x = 0;
		myPos.y = 0;
		//setup Seed Positions - these are hard coded with ID
		seed1Pos.x = 0;
		seed1Pos.y = 0;
		seed2Pos.x = 0;
		seed2Pos.y = 8.0;
	}

	//executed on successfull message send
	void message_tx_success()
	{
		//set_color(RGB(1,0,0));
		msrx=1;
	}

	//sends message at fixed rate
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

	//receives message
	void message_rx(message_t *message, distance_measurement_t *distance_measurement){
		//We do message handling in here as things appear to behave better
		// that is, when a message is rx-ed, it's handled right away.
		//This appears to be an interrupt function

		if(GRAD){
			//Read in messages
			old1 = message->data[0];
			old2 = message->data[1];
			//First gradient if the new one is bigger than the one on record
			if(old1 < hopcounts.data1){
				// the one on record becomes new +1
				hopcounts.data1 = old1+1;
				// bound the hopcount to the max hopcount value
				if(hopcounts.data1 > maxHopVal){
					hopcounts.data1 = maxHopVal;
				}
			}
			//same code as above except for the second gradient.
			if(old2 < hopcounts.data2){
				hopcounts.data2 = old2+1;
				if(hopcounts.data2 > maxHopVal){
					hopcounts.data2 = maxHopVal;
				}
			}
		}
		//message handling for smoothing prop.
		else if(SMOOTH){
			//Read in messages
			old1 = message->data[0];
			old2 = message->data[1];
			//Smooth Hops 1.
			//Add new message to the hops. Do other math elsewhere
			sumhops1 = sumhops1 + double(old1);
			//increment counter only when the message is actually rx-ed. Nowhere else
			//makes sense for this.
			counts1++;

			//Hops 2 here. Repeat of above.
			sumhops2 = sumhops2 + double(old2);
			//increment counter only when the message is actually rx-ed. Nowhere else
			//makes sense for this.
			counts2++;
		}

		out_message.data[2] = message->data[2];
		rxed=1;
	}
};
