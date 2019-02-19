#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include <stdio.h>

//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm
// Copy from local while on local to remote scp /tmp/file user@example.com:/home/name/dir

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4

enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};

struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};

//Function declarations
int setup_imu();
void read_in_params();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_keyboard();
void update_Time();
void safety_check(Keyboard* keyboard);
void init_motor(uint8_t channel);
void init_pwm();
void set_PWM(uint8_t channel, float time_on_us);
void pid_update(float pitch, float roll, Keyboard* keyboard);
void keypress_check(Keyboard* keyboard);

//variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw = 0;
float pitch_angle = 0;
float roll_angle = 0;
float imu_diff = 0.0;
float timer;
long timer_now;
long timer_old;

//New variables for comp. filter
float pitch_filt_now = 0.0;
float roll_filt_now = 0.0;

float pitch_filt_old = 0.0;
float roll_filt_old = 0.0;

float roll_int = 0.0;
float pitch_int = 0.0;

//heartbeat and pausing stuff
int hb_new = 0;
int hb_old = 0;
Keyboard* shared_memory; 
int run_program=1;
bool pauseMotors = false;

// motor pwm 
int pwm;

//Pitch PID controller 
// float m02 = 0;
// float m13 = 0;
float m0 = 0;
float m1 = 0;
float m2 = 0;
float m3 = 0;
float old_pitch = 0.0;
float i_error = 0.0;

//Tunable Parameters - Pitch
float P = 7;
float I = 0.01;
float D = 300.0;
float PWM_MAX = 1650;
float A_CONST = 0.0025;
float MAX_I = 50.0;
bool printing = true;
bool pauser = false;

//File Stuff
FILE *pFile;


// float P = 8.5;
// float I = 0.008;
// float D = 260.0;
// float PWM_MAX = 1650;
// float A_CONST = 0.005;
// float MAX_I = 100.0;
// bool printing = true;

// //Tunable Parameters - Roll
// float P = 12.0;
// float I = 0.003;
// float D = 180.0;
// float PWM_MAX = 1600;
// float A_CONST = 0.005;
// float MAX_I = 50.0;100
// bool printing = fal100

//when ctrl+c pressed,100
void trap(int signal){
  if(signal == 1){
    printf("Kill Button Death\n\r");
  } 
  if(signal == 2){
    printf("Heartbeat Death\n\r");
  }
  if(signal == 3){
    printf("Gyrorate Death\n\r");
  }
  if(signal == 4){
    printf("Roll or Pitch Death\n\r");
  }
  
  printf("ending program\n\r");
  set_PWM(0, 1000);
  set_PWM(1, 1000);
  set_PWM(2, 1000);
  set_PWM(3, 1000);
  run_program=0;
}
 
int main (int argc, char *argv[]){
  //Init Functions 
  setup_imu();
  init_pwm();
  init_motor(0);
  init_motor(1);
  init_motor(2);
  init_motor(3);
  delay(1000);
   //For reading in the parameters at startup
  read_in_params();
  calibrate_imu();
  setup_keyboard();
  signal(SIGINT, &trap);
 
  while(run_program == 1){
    //to refresh values from shared memory first 
    //Keyboard keyboard=*shared_memory;
    //Run the safety check function 
    safety_check(shared_memory);
    keypress_check(shared_memory);
    read_imu();      
    update_filter();
    
    //If motors paused
    if(pauseMotors){
      if(pauser){
        printf("Motors Paused\n\r");
        set_PWM(0, 1000);
        set_PWM(1, 1000);
        set_PWM(2, 1000);
        set_PWM(3, 1000);
        pauser = false;
      }
    }
    else{
      //Get roll and pitch with atan2 
      // y-z is roll
      roll_angle = (atan2(imu_data[3],-imu_data[5]) * (360.0/(2*M_PI))) - roll_calibration ;
      // x-z is pitch  
      pitch_angle = -(atan2(imu_data[4],-imu_data[5]) * (360.0/(2*M_PI))) + pitch_calibration;
     
      pid_update(pitch_filt_now, roll_filt_now, shared_memory);
      pauser = true;
    }
  }
  return 0;
}

void calibrate_imu(){
  //Read the IMU n times; reqs says set n to 1000 
  int n = 1000;

  //Local variables for calibration only 
  float temp_data[6];

  //Zero Out the variable 
  for(int i = 0; i < 6; i++){
    temp_data[i] = 0;
  }

  //Average
  for(int i = 0; i < n; i++){
    //Read the IMU
    read_imu();

    //Capture the baseline values 
    temp_data[0] += imu_data[0]; //x
    temp_data[1] += imu_data[1]; //y
    temp_data[2] += imu_data[2]; //z

    //roll_calibration y-z is roll
    temp_data[3] += atan2(imu_data[3],-imu_data[5]);

    //pitch_calibration x-z is pitch  
    temp_data[4] += atan2(imu_data[4],-imu_data[5]);
  }  

  //Update the global constants, profit
  x_gyro_calibration = temp_data[0] / float(n);
  y_gyro_calibration = temp_data[1] / float(n);
  z_gyro_calibration = temp_data[2] / float(n);
  pitch_calibration  = (temp_data[4] / float(n)) * (360.0/(2*M_PI));
  roll_calibration   = (temp_data[3] / float(n)) * (360.0/(2*M_PI));
  // accel_z_calibration = temp_data[5] / float(n);
}

void read_imu(){
  int address=0x3B;//complete: set address value for accel x value 
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  
  // --- X Accel --- 
  //read in data
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  //convert 2 complement
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]= float(vw)/16384.0;//complete: convert vw from raw values to "g's"
  
  // --- Y ACCEL --- 
  address=0x3D;//complete: set address value for accel y value
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4] = float(vw) / 16384.0;//complete: convert vw from raw valeus to "g's"
  
  // --- Z ACCEL --- 
  address=0x3F;//complete: set addres value for accel z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=float(vw)/16384.0;//complete: convert vw from raw values to g's
  
  // --- X GYRO --- 
  address=0x43;//complete: set addres value for gyro x value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=-x_gyro_calibration+(float(vw)/32768.0)*500.0;////complete: convert vw from raw values to degrees/second
  
  // --- Y GYRO ---
  address=0x45;//complete: set addres value for gyro y value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[1]=-y_gyro_calibration+(float(vw)/32768.0)*500.0;////complete: convert vw from raw values to degrees/second
  //imu_data[1]=(float(vw)/32768.0)*500.0;////complete: convert vw from raw values to degrees/second

  // --- Z GYRO ---
  address=0x47;////complete: set addres value for gyro z value;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=-z_gyro_calibration+(float(vw)/32768.0)*500.0;////complete: convert vw from raw values to degrees/second
  //imu_data[2]=(float(vw)/32768.0)*500.0;////complete: convert vw from raw values to degrees/second
}

void update_filter(){
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;

  //compute time since last execution
  imu_diff = time_curr - time_prev;           
  
  //check for rollover
  if(imu_diff<=0) {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  //comp. filter for roll
  roll_int = roll_int + imu_data[1]*imu_diff;
  roll_filt_now = (roll_angle*A_CONST) + (1.0-A_CONST)*(-imu_data[1]*imu_diff+roll_filt_old);
  roll_filt_old = roll_filt_now;

  //comp. filter for pitch
  pitch_int = pitch_int + imu_data[0]*imu_diff;
  pitch_filt_now = (pitch_angle*A_CONST) + (1.0-A_CONST)*(imu_data[0]*imu_diff+pitch_filt_old);
  pitch_filt_old = pitch_filt_now;
}

int setup_imu(){
  wiringPiSetup ();
  
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}

void setup_keyboard(){
  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

void update_Time(){
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  timer_now=te.tv_nsec;

  //compute time since last execution
  timer = timer_now - timer_old;           
            
  //check for rollover
  if(timer<=0){
    timer+=1000000000;
  }
  //convert to seconds
  timer = timer / 1000000000;
}

void safety_check(Keyboard* keyboard){
  //if space, call trap
    if(keyboard->keypress == 32){
      trap(1);
    }
  //if no heart beat, start timer 
  hb_new = keyboard->sequence_num;
  //check heartbeat 
  if(hb_new == hb_old){
    update_Time();
    //check if 0.25s have passed
    if(timer >= 0.5){
      trap(2);
    }    
  }
  //Check gyrorate error 
  else if(abs(imu_data[0]) > 300.0 || abs(imu_data[1]) > 300.0 || abs(imu_data[2]) > 300.0){
      update_Time();
      //check if 0.10s have passed
      if(timer >= 0.10){
        trap(3);
      }    
  }
  //Roll or Pitch angle 
  else if(abs(pitch_filt_now) > 45.0 || abs(roll_filt_now) > 45.0 ){
      update_Time();
      //check if 0.10s have passed
      if(timer >= 0.10){
        trap(4);
      }   
  }
  //Heartbeat updated, no space, no ctrl+c, no roll, pitch, gyro errors, let run. 
  else{
      hb_old = hb_new; 
      //get time 
      timespec_get(&te,TIME_UTC);
      timer_old = te.tv_nsec; 
  }
}

void init_motor(uint8_t channel){
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1200;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	 time_on_us=1000;
	 off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

}

void set_PWM(uint8_t channel, float time_on_us){
  if(run_program==1){
    if(time_on_us>PWM_MAX){
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<1000){
      time_on_us=1000;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void init_pwm(){
    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
     
    }
    else
    {
  
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void pid_update(float pitch, float roll, Keyboard* keypad){
  float setpoint = int((float(keypad->pitch)*(-0.1786))+22.86);
  //Error Computation 
  float pitch_err = P*(setpoint - pitch);
  
  //Velocity (Derivtive Computation)
  float d_err = pitch - old_pitch;
  old_pitch = pitch;

  //I term integration 
  i_error += I*pitch_err;

  //Integrator wind up prevention
  if(i_error > MAX_I){
    i_error = MAX_I;
  }
  else if(i_error < -MAX_I){
    i_error = -MAX_I;
  }

  float th = float(keypad->thrust)*-1.79 + 1629.0;

  m0 = th - pitch_err + D*d_err - i_error;
  m2 = th - pitch_err + D*d_err - i_error;
  m1 = th + pitch_err - D*d_err + i_error;
  m3 = th + pitch_err - D*d_err + i_error;

  // NEW set 0, 2
  set_PWM(0,m0);
  set_PWM(2,m2);

  // // NEW set 1,3
  set_PWM(1,m1);
  set_PWM(3,m3);
  
  //Print
  pFile = fopen("data.txt","a+");
  fprintf(pFile, "%f,%f\n\r", setpoint, pitch_filt_now);
  fclose(pFile);
}

void read_in_params(){
  float input;
  bool reading = true;
  while(reading){
    puts("Param Selection:\n\r 0 for P\n\r 1 for I\n\r 2 for D\n\r 3 for MaxPWM\n\r 4 for A\n\r 5 for Neutral\n\r 6 for Max_I\n\r 7 for printing\n\r 8 to exit/Default\n\r");
    scanf("%f",&input);
    if(input == 0.0){
      scanf("%f", &P);
      printf("P Set to: %f\n\r",P);
    }
    else if(input == 1.0){
      scanf("%f", &I);
      printf("I Set to: %f\n\r",I);
    }
    else if(input == 2.0){
      scanf("%f", &D);
      printf("D Set to: %f\n\r",D);
    }
    else if(input == 3.0){
      scanf("%f", &PWM_MAX);
      printf("PWM_MAX Set to: %f\n\r",PWM_MAX);
    }
    else if(input == 4.0){
      scanf("%f", &A_CONST);
      printf("A Set to: %f\n\r",A_CONST);
     }
    // else if(input == 5.0){
    //   scanf("%f", &NEUTRAL_POWER);
    //   printf("Neutral Speed Set to: %f\n\r",NEUTRAL_POWER);
    //}
    else if(input == 6.0){
      scanf("%f", &MAX_I);
      printf("Max I Set to: %f\n\r",MAX_I);
    }
    else if(input == 7.0){
      puts("BOOL: 1 or 0 only\n\r");
      scanf("%d", &printing);
      printf("Printing Set to: %f\n\r",printing);
    }
    else if(input == 8.0){
      reading = false;
      puts("Goodbye\n\r");
    }
    else{
      puts("Not Correct Inputs.\n\r");
    }
  }
  return;
}

void keypress_check(Keyboard* keypad){
  //Check Keypresses for calibrate
  if(keypad->keypress == 35){
    calibrate_imu();
    printf("Calibrated IMU\n\r");
  }
  //Pause motors
  if(keypad->keypress == 33 && !pauseMotors){
    pauseMotors = true;
    printf("MOTORS PAUSED\n\r");
  }
  //Unpause motors 
  if(keypad->keypress == 34 && pauseMotors){
    pauseMotors = false;
    printf("MOTORS UNPAUSED\n\r");
  }
}