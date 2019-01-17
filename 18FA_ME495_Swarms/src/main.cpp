#pragma warning(disable:4996)

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include "robot.h"
#include "kilobot.cpp"
#include "setup.cpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <fstream>


#define SIMPLEBMP_OPENGL
#include "simplebmp.h"
using namespace std;

#define buffer_size 1000000
#define channels 2
//#define delay 10 //delay between time steps, use if program is too fast
#define windowWidth 600 //display window
#define windowHeight 700 //display window
#define comm_noise_std 5 //standard dev. of sensor noise
#define PI 3.14159265358979324
#define twicePi  2 * PI
#define radius 16 //radius of a robot
#define p_control_execute .99 // probability of a controller executing its time step
#define SKIPFRAMES 0
#define shuffles 20
#define circledef 30
// Global vars.
double time_sim;  //simulation time
double zoom, view_x, view_y; //var. for zoom and scroll
bool takesnapshot;
int num_robots = ROBOT_COUNT; //number of robots running

robot** robots;//creates an array of robots
int* safe_distance;
int* order;

int delay = 0;
int draw_delay=32;
FILE *results;

char log_buffer[255];
char log_file_buffer[buffer_size];

int light_center[2]={LIGHT_CENTER_X,LIGHT_CENTER_Y};


bool log_debug_info = true;
char log_file_name[255] = "simulation.log";
bool showscene = true;

char shape_file_name[255] = "";


int total_secs;
int timelimit = SIMULATION_TIME * 60;
char rt[100];

double ch[radius];

int arena_width = ARENA_WIDTH;
int arena_height = ARENA_HEIGHT;

int snapshot = 60;
int snapshotcounter = 0;

bool last = false;
bool write_final = false;

unsigned int seed = 0;

void strcpy_safe(char *m, int l, char *s)
{
	for (int i = 0; i <= l && s; i++)
		*m = *s;
}

void log_info(char *s)
{
	static char *m = log_file_buffer;
	if (s)
	{
		int l = strlen(s) + 1;
		strcpy(m, s);
		m += l - 1;
	}
	if (m - log_file_buffer >= buffer_size-255 || !s)
	{
		results = fopen(log_file_name, "a");
		fprintf(results, "%s", log_file_buffer);
		fclose(results);
		m = log_file_buffer;
	}
}
//check to see if motion causes robots to collide
int find_collisions(int id, double x, double y)
{
	double two_r = 2 * radius;
	int i;
	if (x <= radius || x >= arena_width - radius || y <= radius || y >= arena_height - radius) return 1;
	double x_ulim = x + two_r;
	double x_llim = x - two_r;
	double y_ulim = y + two_r;
	double y_llim = y - two_r;
	for (i = 0;i < num_robots;i++)
	{
		if (i != id)
		{
			if (safe_distance[id*num_robots+i])
			{
				safe_distance[id*num_robots + i]--;
			}
			else
			{
				double dist_x = x - robots[i]->pos[0];
				double dist_y = y - robots[i]->pos[1];
				if (x_ulim > robots[i]->pos[0] && x_llim<robots[i]->pos[0] &&
					y_ulim>robots[i]->pos[1] && y_llim < robots[i]->pos[1]) //if not in the sqare limits, i dont even check the circular ones
				{

					double distance = sqrt(dist_x*dist_x + dist_y * dist_y);
					if (distance < two_r)
					{
						return 1;
					}
				}
				else
				{
					double bd = 0;
					if (fabs(dist_x)>fabs(dist_y))
					{
						bd = fabs(dist_x);
					}
					else
					{
						bd = fabs(dist_y);
					}
					if (bd > two_r+20)
					{
						double speed = robots[id]->speed + robots[i]->speed;
						if (speed > 0)
						{
							safe_distance[id*num_robots + i] = (int)((bd - (two_r + 20)) / speed);
						}
						else
						{
							safe_distance[id*num_robots + i] = 1000000;
						}
						safe_distance[i*num_robots + id] = safe_distance[id*num_robots + i];
					}
				}
			}
		}
	}
	return 0;
}

void save_bmp(const char *fileName)
{

	// The width and the height, would be the width
	// and height of your current scene.

	SimpleBMP bmp(windowWidth, windowHeight);

	bmp.glReadPixels();
	printf("snapshot saved to %s\n",fileName);
	bmp.save(fileName);
}

// -- THIS FUNCTION --
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
	// -- ^^THIS FUNCTION^^ --

bool run_simulation_step()
{
	measure_metric();
	static int lastrun = 0;
	lastrun++;

	total_secs = lastrun / SECOND;

	int secs = total_secs % 60;
	int mins = (total_secs / 60) % 60;
	int hours = total_secs / 3600;
	sprintf(rt, "%02d:%02d:%02d", hours, mins, secs);

	int i, j;

	double rotation_step = .05;//motion step size
							   //run a step of most or all robot controllers
	for (i = 0;i < num_robots;i++)
	{
		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX))
		{
			robots[i]->robot_controller();
		}
	}

	//update angle to light for robots
	for(i=0;i<num_robots;i++)
	{

  //	robots[i]->angle_to_light=fmod(atan2(light_center[1]-robots[i]->pos[1],light_center[0]-robots[i]->pos[0])-robots[i]->pos[2]+PI,2*PI)-PI;
	robots[i]->pos[2]=fmod(robots[i]->pos[2],2*PI);
	robots[i]->angle_to_light=fmod(atan2(light_center[1]-robots[i]->pos[1],light_center[0]-robots[i]->pos[0])-robots[i]->pos[2]+PI,2*PI)-PI   ;
  //printf("in main angle is %f\n\r",robots[i]->angle_to_light);

	}


	int seed;
	seed = (rand() % shuffles) * num_robots;
	//let robots communicate
	for (i = 0;i < num_robots;i++)
	{
		int index = order[seed + i];
		robot *rs = robots[index];
		//if robot wants to communicate, send message to all robots within distance comm_range
		void *msg = rs->get_message();
		if (msg)
		{
			for (j = 0;j < num_robots;j++)
			{
				robot *rd = robots[j];
				if (j != index)
				{
					double range = rs->comm_out_criteria(rd->pos[0], rd->pos[1], safe_distance[index * num_robots + j]);
					if (range)
					{
						float theta=0;
						theta=atan2(rs->pos[1]-rd->pos[1],rs->pos[0]-rd->pos[0])-rd->pos[2];


						if (rd->comm_in_criteria(rs->pos[0], rs->pos[1], range,theta, msg))
						{
							rs->received();
							//break;
						}
					}
				}
			}
		}
	}

	seed = (rand() % shuffles) * num_robots;
	//move robots
	for (i = 0;i < num_robots;i++)
	{
		int index = order[seed + i];
		robot *r = robots[index];

		double t = r->pos[2];
		double s = 0;
		switch (r->motor_command)
		{
		case 1:
		{
			t += r->motor_error;
			s = r->speed;
			break;
		}
		case 2:
		{
			t += rotation_step;
			s = r->speed;
			if (r->pos[2] > twicePi)
			{
				r->pos[2] -= twicePi;
			}
			break;
		}
		case 3:
		{
			t -= rotation_step;
			s = r->speed;
			if (r->pos[2] < 0)
			{
				r->pos[2] += twicePi;
			}
			break;
		}
		}
		double temp_x = s*cos(t) + r->pos[0];
		double temp_y = s*sin(t) + r->pos[1];
		if (find_collisions(index, temp_x, temp_y) == 0)
		{
			r->pos[0] = temp_x;
			r->pos[1] = temp_y;
		}
		r->pos[2] = t;
	}
	static int lastsec =- 1;
	bool result = false;
	if(lastrun%draw_delay==0)
		return true;
	return false;
}

// Drawing routine.
void draw_scene(void)
{
	static int snapshottaken = 0;
	static bool draw = false;
	//draws the arena

	draw = run_simulation_step();


	if(draw)
	{
		glColor4f(0, 0, 0, 0);
		glRectd(0, 0, arena_width, arena_height);

		glutSetWindowTitle(rt);
		glEnable(GL_LINE_SMOOTH);
		glLineWidth(1.0);
		glBegin(GL_LINES);
		for (int i = 0; i <= radius; i++)
		{
			for (int j = 0;j < num_robots;j++)
			{
				glColor4f((GLfloat)robots[j]->color[0], (GLfloat)robots[j]->color[1], (GLfloat)robots[j]->color[2], 1.0);
				glVertex2f((GLfloat)(robots[j]->pos[0]-i), (GLfloat)(robots[j]->pos[1]-ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] -i), (GLfloat)(robots[j]->pos[1] + ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] - ch[i]));
				glVertex2f((GLfloat)(robots[j]->pos[0] + i), (GLfloat)(robots[j]->pos[1] + ch[i]));
			}
		}
		for (int j = 0;j < num_robots;j++)
		{
			glBegin(GL_LINES);
			glColor4f(0, 0, 0, 1.0);
			glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
			glVertex2f((GLfloat)(robots[j]->pos[0] + cos(robots[j]->pos[2])*radius), (GLfloat)(robots[j]->pos[1] + sin(robots[j]->pos[2])*radius));
			if (robots[j]->dest[0] != -1)
			{
				glBegin(GL_LINES);
				glColor4f(1, 1, 1, 1.0);
				glVertex2f((GLfloat)robots[j]->pos[0], (GLfloat)robots[j]->pos[1]);
				glVertex2f((GLfloat)robots[j]->dest[0], (GLfloat)robots[j]->dest[1]);
			}
		}
		glEnd();
		glFlush();

		if (takesnapshot)
		{
			snapshottaken++;
			char file[100];
			sprintf(file, "IMG%05i.bmp", snapshottaken++);
			save_bmp(file);
			takesnapshot = false;
		}

		glutSwapBuffers();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	}

	if (last)
	{
		log_info(NULL);
		exit(0);
	}
	if (total_secs >= timelimit)
	{
		last = true;
	}
}

// Initialization routine.
void setup(void)
{
	for (int i = 0;i < num_robots;i++)
		for (int j = 0;j < shuffles;j++)
			order[i + num_robots*j] = i;

	for (int i = 0;i < num_robots - 1;i++)
		for (int j = 0;j < shuffles;j++)
		{
			int index = j*num_robots + i;
			int r = index + rand() % (num_robots - i);
			int p = order[index];
			order[index] = order[r];
			order[r] = p;
		}
	for (int i = 0;i < num_robots;i++)
		for (int j = 0;j < num_robots;j++)
			safe_distance[i * num_robots + j] = 0;
}

// OpenGL window reshape routine.
void resize_window(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Keyboard input processing routine.
void key_input(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'w'://up
		view_y += 100;
		break;
	case 'a'://up
		view_x -= 100;
		break;
	case 's'://up
		view_y -= 100;
		break;
	case 'd'://up
		view_x += 100;
		break;
	case '-':
		zoom = zoom*1.1;
		break;
	case '+':
		zoom = zoom*0.9;
		break;
	case '1':
		if (delay>0)
			delay--;
		printf("busy delay %d\n\r",delay);
		break;
	case '2':
		delay++;
		printf("busy delay %d\n\r",delay);
		break;
	case '3':
		if (draw_delay>1)
			draw_delay--;
		printf("draw delay %d\n\r",draw_delay);
		break;
	case '4':
		draw_delay++;
		printf("draw delay %d\n\r",draw_delay);
		break;
	case ' ':
		takesnapshot=true;
		break;
	default:
		break;
	}
}

void on_idle(void) {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-zoom + view_x, view_x, -zoom + view_y, view_y, 0.0f, 1.0f);
	std::this_thread::sleep_for(std::chrono::milliseconds(delay));
	glutPostRedisplay();
}

// Main routine.
int main(int argc, char **argv)
{
	for (int i = 0;i < argc-1;i++)
	{
		if (strcmp(argv[i],"/r")==0)
		{
			num_robots = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/l") == 0)
		{
			log_debug_info = argv[i + 1][0]=='y';
		}
		if (strcmp(argv[i], "/d") == 0)
		{
			showscene = argv[i + 1][0] == 'y';
		}
		if (strcmp(argv[i], "/aw") == 0)
		{
			arena_width = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/ah") == 0)
		{
			arena_height = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/t") == 0)
		{
			timelimit = stoi(argv[i + 1]) ;
		}
		if (strcmp(argv[i], "/f") == 0)
		{
			strcpy_safe(log_file_name,255, argv[i + 1]);
		}
		if (strcmp(argv[i], "/ss") == 0)
		{
			snapshot = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/seed") == 0)
		{
			seed = stoi(argv[i + 1]);
		}
		if (strcmp(argv[i], "/shape") == 0)
		{
			strcpy_safe(shape_file_name, 255, argv[i + 1]);
		}
	}

	robots = (robot **)malloc(num_robots * sizeof(robot *));//creates an array of robots
	safe_distance = (int *) malloc(num_robots * num_robots * sizeof(int));
	order = (int *) malloc(shuffles * num_robots * sizeof(int));
	//seed random variable for different random behavior every time
	unsigned int t = 0;

	if (seed)
	{
		t = seed;
	}
	else
	{
		t= (unsigned int) time(NULL);
	}

	sprintf(log_buffer, "random seed: %d\n", t);

	log_info(log_buffer);
	srand(t);

	//set the simulation time to 0
	time_sim = 0;

	//inital zoom and scroll positions
	zoom = arena_width;
	view_x = arena_width;
	view_y = arena_height;

	//place robots
	float robot_pos[ROBOT_COUNT][4];

	setup_positions(robot_pos);

	for (int i=0; i<ROBOT_COUNT; i++)
	{
		robots[i] = new mykilobot();
		robots[i]->robot_init(robot_pos[i][0], robot_pos[i][1], robot_pos[i][2]);
		robots[i]->id=(int)robot_pos[i][3];
	}
	setup();

	//do some open gl stuff

	for (int i = 0;i < radius;i++)
	{
		ch[i] = sqrt(radius*radius - i*i);
	}

	if (showscene)
	{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
		glutInitWindowSize(windowWidth, windowHeight);
		glutInitWindowPosition(0, 0);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
		glClearColor(1.0, 1.0, 1.0, 0.0);
		glutCreateWindow("Kilobot simulator");

		glutDisplayFunc(draw_scene);
		glutReshapeFunc(resize_window);
		glutIdleFunc(on_idle);
		glutKeyboardFunc(key_input);
		glutMainLoop();
	}
	else {
		while (total_secs<timelimit)
		{
			run_simulation_step();
		}

		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
		glutInitWindowSize(windowWidth, windowHeight);
		glutInitWindowPosition(0, 0);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
		glClearColor(1.0, 1.0, 1.0, 0.0);
		glutCreateWindow("Kilobot simulator");

		glutDisplayFunc(draw_scene);
		glutReshapeFunc(resize_window);
		glutIdleFunc(on_idle);
		glutKeyboardFunc(key_input);
		glutMainLoop();
	}
	return 0;
}
