#include <iostream>
#include <opencv2/opencv.hpp>
#include "vive.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/time.h>


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>

#define GROUPS   2
#define RANGE    1200
#define COMPLETE 15

#define PROYECTION 100.0
#define PI 3.14159265359
#define ROTATION PI/2
#define CONVERSION 100

#define FRONT 0
#define BACK 2

using namespace std;
using namespace cv;

cv::Mat camera_matrix;
cv::Mat dist_coeffs;
cv::Point3d model3D[4];

float data[VALUES];

int open_uart()
{
	int uart0_filestream = -1;
	uart0_filestream = open("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
		return uart0_filestream;
	}
	static struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B230400 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = ICANON;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);	
	printf("UART opened\n");
	return uart0_filestream;
}
 
void close_uart(int uart0_filestream)
{
	close(uart0_filestream);	
}

unsigned char *clean_instruction (unsigned char *instruction)
{
	unsigned char *result;
	while (*instruction && *instruction!='{')
	{
		instruction++;
	}
	if (!*instruction) return NULL;
	result=++instruction;
	while (*instruction && *instruction!='}')instruction++;
	if (!*instruction) return NULL;
	*instruction='\0';
	//cout << result;
	return result;	
}

bool read_uart(unsigned char *rx_buffer, int uart0_filestream)
{
	// Read up to 255 characters from the port if they are there
	static int used = 0;
	int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
	if (rx_length>0 )
	{
		rx_buffer[rx_length]=0;
		*rx_buffer=*rx_buffer?*rx_buffer:' ';
		return true;
	}
	return false;
}

bool parse_instruction(float *data,bool *read,unsigned char *instruction)
{
	char *start = (char *) instruction;
	char *end   = (char *) instruction;
	for (int i=0;i<VALUES;i++)
	{
		while (*end && *end!=',')end++;
		if (!*end) return false;
		*end='\0';
		data[i]= atof(start)/CONVERSION;
		start=++end;
	}
	int c = atoi(start);
	for (int i=0;i<SENSORS;i++)
	{
		read[i]= c & (1<<i);
	}
	
	return c==COMPLETE;
}

bool process_instruction(std::vector<cv::Point2d> *image_points, unsigned char *instruction)
{
	bool read[VALUES];
	float cx = 0,cy = 0;
	
	
	if (!parse_instruction(data,read,instruction)) return false;
	
	std::vector<cv::Point2d> sensors;
	for (int i=0;i<SENSORS;i++)
	{
		int xi=i*2;
		int yi=i*2+1;
		image_points->push_back(cv::Point2d(data[xi], data[yi]));

	}
	return true;
}

float acos2(float a, float h)
{
	return (acos(a / h));
}

void to_pose(double pos[3], double orientation[4], cv::Mat Rvec, cv::Mat Tvec) 
{
  
    //check if paremeters are valid
    bool invalid=false;
    for (int i=0;i<3 && !invalid ;i++)
    {
        if (Tvec.at<float>(i,0)!=-999999) invalid|=false;
        if (Rvec.at<float>(i,0)!=-999999) invalid|=false;
    }
    
    // calculate position vector
    pos[0] = -Tvec.ptr<float>(0)[0];
    pos[1] = -Tvec.ptr<float>(0)[1];
    pos[2] = +Tvec.ptr<float>(0)[2];
    
    // now calculare orientation quaternion
    cv::Mat Rot(3,3,CV_32FC1);
    cv::Rodrigues(Rvec, Rot);
    
    // calculate axes for quaternion
    double stAxes[3][3];
    // x axis
    stAxes[0][0] = -Rot.at<float>(0,0);
    stAxes[0][1] = -Rot.at<float>(1,0);
    stAxes[0][2] = +Rot.at<float>(2,0);
    // y axis
    stAxes[1][0] = -Rot.at<float>(0,1);
    stAxes[1][1] = -Rot.at<float>(1,1);
    stAxes[1][2] = +Rot.at<float>(2,1);    
    // for z axis, we use cross product
    stAxes[2][0] = stAxes[0][1]*stAxes[1][2] - stAxes[0][2]*stAxes[1][1];
    stAxes[2][1] = - stAxes[0][0]*stAxes[1][2] + stAxes[0][2]*stAxes[1][0];
    stAxes[2][2] = stAxes[0][0]*stAxes[1][1] - stAxes[0][1]*stAxes[1][0];
    
    // transposed matrix
    double axes[3][3];
    axes[0][0] = stAxes[0][0];
    axes[1][0] = stAxes[0][1];
    axes[2][0] = stAxes[0][2];
    
    axes[0][1] = stAxes[1][0];
    axes[1][1] = stAxes[1][1];
    axes[2][1] = stAxes[1][2];  
    
    axes[0][2] = stAxes[2][0];
    axes[1][2] = stAxes[2][1];
    axes[2][2] = stAxes[2][2];    
    
    // Algorithm in Ken Shoemake's article in 1987 SIGGRAPH course notes
    // article "Quaternion Calculus and Fast Animation".
    double fTrace = axes[0][0]+axes[1][1]+axes[2][2];
    double fRoot;
      
    if ( fTrace > 0.0 )
    {
		// |w| > 1/2, may as well choose w > 1/2
		fRoot = sqrt(fTrace + 1.0);  // 2w
		orientation[0] = 0.5*fRoot;
		fRoot = 0.5/fRoot;  // 1/(4w)
		orientation[1] = (axes[2][1]-axes[1][2])*fRoot;
		orientation[2] = (axes[0][2]-axes[2][0])*fRoot;
		orientation[3] = (axes[1][0]-axes[0][1])*fRoot;
    }
    else
    {
		// |w| <= 1/2
		static unsigned int s_iNext[3] = { 1, 2, 0 };
		unsigned int i = 0;
		if ( axes[1][1] > axes[0][0] )
			i = 1;
		if ( axes[2][2] > axes[i][i] )
			i = 2;
		unsigned int j = s_iNext[i];
		unsigned int k = s_iNext[j];

		fRoot = sqrt(axes[i][i]-axes[j][j]-axes[k][k] + 1.0);
		double* apkQuat[3] = { &orientation[1], &orientation[2], &orientation[3] };
		*apkQuat[i] = 0.5*fRoot;
		fRoot = 0.5/fRoot;
		orientation[0] = (axes[k][j]-axes[j][k])*fRoot;
		*apkQuat[j] = (axes[j][i]+axes[i][j])*fRoot;
		*apkQuat[k] = (axes[k][i]+axes[i][k])*fRoot;
    }    
}

bool to_RPY (std::vector<cv::Point2d> image_points, std::vector<cv::Point3d> model_points)
{
	double pos[3], orientation[4];

	cv::Mat rotation_vector; // Rotation in axis-angle form
	cv::Mat translation_vector;	

	// Determine the yaw as the angle of the line between center and front ponints in 2d
	position->yaw = atan2(image_points[FRONT].x - image_points[BACK].x, image_points[FRONT].y - image_points[BACK].y) ;
	
	// Solve for pose
	cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
/*	
	to_pose(pos, orientation, rotation_vector, translation_vector);

	position->x = pos[0];
	position->y = pos[1];
	position->z = pos[2];
	*/	
	
	position->x = -translation_vector.ptr<double>(0)[0];
	position->y = -translation_vector.ptr<double>(0)[1];
	position->z = +translation_vector.ptr<double>(0)[2];
	return true;
}

int main(int argc, char **args)
{
	double startTime, endTime, lastRead;
	init_shared_memory();
	position->version=0;
	
	std::vector<cv::Point3d> model_points;
	model_points.push_back(cv::Point3d(0.0f, -50.0f, 0.0f));
	model_points.push_back(cv::Point3d(50.0f, 0.0f, 0.0f));
	model_points.push_back(cv::Point3d(0.0f, 50.0f, 0.0f));
	model_points.push_back(cv::Point3d(-50.0f, 0.0f, 0.0f));

	// Camera internals
	double focal_length = RANGE; // Approximate focal length.
	Point2d center = cv::Point2d(RANGE / 2, RANGE / 2);
	camera_matrix = (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

	//cout << "Camera Matrix " << endl << camera_matrix << endl;
	// Output rotation and translation

	unsigned char instruction[255];
	int uart=open_uart();
	int j=0;
	if (uart<0) return 0;
	lastRead = (float)clock()/CLOCKS_PER_SEC;
	bool r;
	while(1)
	{
		r=false;
		while(read_uart(instruction,uart))
		{
			r=true;
		}
		if (r)
		{
			unsigned char *cleaninstruction = clean_instruction(instruction);
			if (*cleaninstruction)
			{
				std::vector<cv::Point2d> image_points;
				if (process_instruction(&image_points,cleaninstruction))
				{
					if (to_RPY(image_points,model_points))
					{
						position->version++;
					}
				}
			}			
		}
	}
	close_uart(uart);
	return 0;
}