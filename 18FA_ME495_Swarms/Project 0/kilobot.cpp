#pragma once
#include "kilolib.h"

class mykilobot : public kilobot
{
	unsigned char distance;
	message_t out_message;
	int rxed=0;

	int motion=0;
	long int motion_timer=0;

	//High and Low variables
	//initalize these at the middle
	int high = 128;
	int low = 128;

	int msrx=0;
	struct mydata {
		unsigned int data1;
		unsigned int data2;
	};

	//main loop
	void loop()
	{
		//If Message is RX-ed
		if(rxed == 1){
			//rx flag to 0
			rxed == 0;
			// FIRST ROUND CHECK
			if(out_message.data[1] == 0 && out_message.data[2] == 0){
				//Check to ensure ID is not equal
				if(id == out_message.data[0]){
					id = rand()%256;
				}
				// if the robot id is lower than the rx-ed ID
				if(id < out_message.data[0]){
					low = id;
					out_message.data[2] = id; //robot ID is the lowest
					out_message.data[1] = out_message.data[0]; // RX-ed ID is highest
					high = out_message.data[0];
					out_message.data[0] = id; // print the robot's ID from now on
				}
				else{
					high = id;
					out_message.data[1] = id; //robot ID is the highest
					out_message.data[2] = out_message.data[0]; // RX-ed ID is lowest
					low = out_message.data[0];
					out_message.data[0] = id; // print the robot's ID from now on
				}
			}
			else{
				//If the highest message rx-ed is higher than your highest recorded:
				// update your highest value!
				if(high < out_message.data[2]){
					//store highest and don't change the message
					high = out_message.data[2];
				}
				else{
					//If the highest is already stored, put it into the message
					out_message.data[2] = high;
				}
				//If the highest message rx-ed is lower than your lowest recorded:
				// update your lowest value!
				if(low > out_message.data[1]){
					//store lowest and don't change the message
					low = out_message.data[1];
				}
				else{
					//If the lowest is already stored, put it into the message
					out_message.data[1] = low;
				}

				//Check if your id is highest or lowest
				if(id == low){
					//set low light
					set_color(RGB(0,0,1));
				}
				else if(id == high){
					//set high light
					set_color(RGB(0,1,0));
				}
				else{
					//set normal light
					set_color(RGB(1,0,0));
				}
		 }

		}

		//Transmit the message
		out_message.data[0] = id;
		out_message.crc = message_crc(&out_message);
		}

	//executed once at start
	void setup()
	{

		id = rand()%256;
		out_message.type = NORMAL;
		out_message.data[0] = id; // THE ID
		out_message.data[1] = 0; // HIGH
		out_message.data[2] = 0; // LOW
		out_message.crc = message_crc(&out_message);
		set_color(RGB(0,0,1));

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
	void message_rx(message_t *message, distance_measurement_t *distance_measurement)
	{
		distance = estimate_distance(distance_measurement);
		out_message.data[0] = message->data[0];
		out_message.data[1] = message->data[1];
		out_message.data[2] = message->data[2];
		rxed=1;
	}
};
