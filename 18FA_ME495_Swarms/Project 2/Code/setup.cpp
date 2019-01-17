#define ROBOT_SPACING 40

#define ARENA_WIDTH (32*32 + 33*ROBOT_SPACING)*2
#define ARENA_HEIGHT (32*32 + 33*ROBOT_SPACING)*2

#define LIGHT_CENTER_X ARENA_WIDTH/2
#define LIGHT_CENTER_Y ARENA_HEIGHT/2

#define SIMULATION_TIME 180 //in seconds
#define PI 3.14159265358979324

// --- VARIABLES TO TOGGLE ROBOT NUMS AND SHELLS HERE ---
//TOGGLE THIS TO SET NUM ROBOTS; Pick 100 or 210
#define ROBOT_COUNT 210
//Number of Groups
#define NUM_GROUPS 3
//TOGGLE TO ENABLE OR DISABLE SHELLS HERE
#define ISSHELLED false

//Magic Distance Function that needs to go everywhere apparently
static double distance(double x1, double y1, double x2, double y2){
	double x = x1 - x2;
	double y = y1 - y2;
	double s = pow(x, 2) + pow(y, 2);
	return sqrt(s);
}

void setup_positions(float robot_pos[ROBOT_COUNT][4]){
		int x = 350;
		int y = 350;
		//assign each robot a random position, centered around light source
		for(int robot_num=0;robot_num<=ROBOT_COUNT;robot_num++){
			//Generate Semi-Random Positions
			robot_pos[robot_num][0]= x + rand()%300 - 150;
			robot_pos[robot_num][1]= y + rand()%300 - 150;

			//Increoment counters etc
			y+=350;
			if(y > 11*350+1){
				y = 350;
				x+=350;
			}

			//Random Angle
			robot_pos[robot_num][2]= ((double)(rand() % 101)/100)*2*PI;

			//Default ID assignment
			robot_pos[robot_num][3]=0;

			//Assign IDs if Shelling and number of groups
			if(robot_num % NUM_GROUPS  == 0 && ISSHELLED){
				robot_pos[robot_num][3]=1;
			}
			else if (robot_num % NUM_GROUPS - 1 == 0 && ISSHELLED && NUM_GROUPS > 2){
				robot_pos[robot_num][3]=2;
			}
		}

}
