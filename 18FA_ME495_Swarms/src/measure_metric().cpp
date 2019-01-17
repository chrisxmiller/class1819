void measure_metric(){
	double nk = 3.0;
	double err = 0.0;
	double printable = 0.0;

	//Light Locs
	double ox = (double) light_center[0];
	double oy = (double) light_center[1];

	//For the II coords
	double xii = 0.0;
	double yii = 0.0;

	//For the JJ coords
	double xjj = 0.0;
	double yjj = 0.0;

	//Nested for loops
	for (int ii = num_robots-1; ii >= 0; ii--){
		for (int jj = num_robots-1; jj >= 0; jj--){
			if (ii != jj){
				//grab the robot positions
				xii = robots[ii]->pos[0];
				yii = robots[ii]->pos[1];

				xjj = robots[jj]->pos[0];
				yjj = robots[jj]->pos[1];

				//if the id of ii is less than jj
				if(robots[ii]->id < robots[jj]->id){
					//straight from the paper
					if(distance(xii,yii,ox,oy) >= distance(xjj,yjj,ox,oy)){
						//increment the error number
						err++;
					}
				}
				//If the id of ii is greater than jj
				if(robots[ii]->id > robots[jj]->id){
					//straight from the paper
					if(distance(xii,yii,ox,oy) <= distance(xjj,yjj,ox,oy)){
						//increment the error number
						err++;
					}
				}
			}
		}
	}
	//Final computation of the measure for this iteration
	//straight from the paper
 	printable = err / (pow(num_robots,2) - pow(NUM_GROUPS,2));

	//Filename stuff
	string robotnum = to_string(ROBOT_COUNT);
	string groupnum = to_string(NUM_GROUPS);
	string ffname = "RC_"+robotnum+"_GN_"+groupnum+".txt";
	char nam[255];
	strncpy(nam,ffname.c_str(),sizeof(nam));

	//Print to a file
	results = fopen(nam,"a");
	fprintf(results, "%f\n",printable);
	fclose(results);
	}
