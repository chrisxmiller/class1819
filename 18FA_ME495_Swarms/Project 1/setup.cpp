void setup_positions(float robot_pos[ROBOT_COUNT][4])
{
	//Seperation distance between robots (on my monitor, they're touching)

	int columns = 32;
	int rows = 32;
	int separation = 40;
	int x = arena_width/2 - (separation*columns)/2;
	int y = arena_height/2 -(separation*rows)/2;
	int last = 0;

	// 1 for HEXAGON 0 for SQUARE.
	int HeX_Toggle = 0;

	//Set their angle to 0 degrees
	for (int i = 0; i < rows; i++){
	  for(int j = 0; j < columns; j++){
	    x = x + separation;
			robot_pos[last+i+j][0] = x;
			robot_pos[last+i+j][1] = y;
			//Theta
			robot_pos[last+i+j][2] = 0.0;
	  }
	  y = y + separation;
		last = last+31;
		//Shape formation. Secret sauce is in hex multiplier below.
		if(i%2 == 0){
	  	x = arena_width/2-(separation*columns)/2-(separation/2)*HeX_Toggle;
		}
		else{
			x = arena_width/2-(separation*columns)/2;
		}

	}
//Pick seedz Robotz with a z
robot_pos[0][3] = 1; 		//seedz1
robot_pos[31][3] = 32;  //seedz2

}
