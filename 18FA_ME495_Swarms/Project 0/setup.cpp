/************************
**** Assignment 1 - setup.cpp
**** EECS 496
**** Name: Connor Bain
************************/

#define ROBOT_COUNT 10

#define ARENA_WIDTH 1000
#define ARENA_HEIGHT 1000

#define SAFE_DIST 0

#define SIMULATION_TIME 180 //in seconds

#include <iostream>

void setup_positions(float robot_pos[ROBOT_COUNT][3])
{
	double distance; // Variable to store the distance calc between bots
  bool collision;  // Flag to store if there is a collision

	// Set robot positions
	for (int i = 0;i < ROBOT_COUNT;i++)
	{
		// For every robot generate an (X,Y,H) 3-tuple
		int myX, myY;

		// Generate (X,Y) within a safe boundary and make sure there are no
		// near by bots.
		collision = true;
		while (collision) {
			collision = false;
			// Generate within an arena safe-area
			myX = rand() % (ARENA_WIDTH - 200) + 100;
			myY = rand() % (ARENA_HEIGHT - 200) + 100;

			// Go through and check each bot for collisions (within 50mm)
			for (int j = 0; j < i; j++) {
				// Calculate the distance between the ith bot and the jth bot
				distance = sqrt(pow((robot_pos[j][0] - myX), 2) + pow((robot_pos[j][1] - myY), 2));
				// If there's a collision, go back and generate a new (X,Y) pair
				if (distance < SAFE_DIST) { collision = true; break; }
			}
		}

		robot_pos[i][0] = myX; // Save the new X value
		robot_pos[i][1] = myY; // Save the new Y value
		robot_pos[i][2]= rand() * 2 * PI / RAND_MAX; //thetas
	}
}
