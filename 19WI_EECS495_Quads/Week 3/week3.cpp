#include <stdio.h>
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
// #include <cstring>
// using namespace std;
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
#define A_CONST          0.02 //Filter constant 
#define PWM_MAX 1300
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4
#define NEUTRAL_POWER 1100
#define P 10

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
  char key_press;
  int heartbeat;
  int version;
};

//Function declarations
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
void setup_keyboard();
void update_Time();
void safety_check(Keyboard keyboard);
void init_motor(uint8_t channel);
void init_pwm();
void set_PWM(uint8_t channel, float time_on_us);
int pid_update(float setpoint, float pitch, float roll);

//global variables
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

//heart beat stuff
int hb_new = 0;
int hb_old = 0;
Keyboard* shared_memory; 
int run_program=1;

// motor pwm 
int pwm;

//when ctrl+c pressed, kill motors
void trap(int signal){
  if(signal == 1){
    printf("Space Pressed! \n\r");
  } 
  if(signal == 2){
    printf("Keyboard Timeout \n\r");
  }
  if(signal == 3){
    printf("Gyrorate Timeout \n\r");
  }
  if(signal == 4){
    printf("Roll or Pitch Timeout \n\r");
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
  calibrate_imu();
  setup_keyboard();
  signal(SIGINT, &trap);

  while(run_program == 1){
    //to refresh values from shared memory first 
    //Keyboard keyboard=*shared_memory;
    //Run the safety check function 
    //safety_check(*shared_memory);
    read_imu();      
    update_filter();

    //Get roll and pitch with atan2 
    // y-z is roll
    roll_angle = (atan2(imu_data[3],-imu_data[5]) * (360.0/(2*M_PI))) - roll_calibration ;
    // x-z is pitch  
    pitch_angle = (atan2(imu_data[4],-imu_data[5]) * (360.0/(2*M_PI))) - pitch_calibration;

    //printf("%f\n\r",pitch_filt_now);
    printf("%f\n\r",pitch_filt_now);
  }
  return 0;
}

void calibrate_imu(){
  //Read the IMU n times; reqs says set n to 1000 
  int n = 1000;

  //Local variables for calibration only 
  float temp_data[6];

  //Averahe
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
  roll_calibration =  (temp_data[4] / float(n)) * (360.0/(2*M_PI));
  pitch_calibration = (temp_data[3] / float(n)) * (360.0/(2*M_PI));
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
  if(imu_diff<=0)
  {
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
  pitch_filt_now = -(pitch_angle*A_CONST) + (1.0-A_CONST)*(imu_data[0]*imu_diff+pitch_filt_old);
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

void safety_check(Keyboard keyboard){
    //if space, call trap
    //   if(keyboard.key_press == ' '){
    //     trap(1);
    //   }
    
    //if no heart beat, start timer 
    //   hb_new = keyboard.heartbeat;
    //   //check heartbeat 
    //   if(hb_new == hb_old){
    //       update_Time();
    //       //check if 0.25s have passed
    //       if(timer >= 0.25){
    //         trap(2);
    //       }    
    //   }

  //Check gyrorate error 
  if(abs(imu_data[0]) > 300.0 || abs(imu_data[1]) > 300.0 || abs(imu_data[2]) > 300.0){
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

float pid_update(float pitch, float roll){
  float setpoint = 0;
    float pitch_1_err =  setpoint - pitch;//motors 0 and 2
    float pitch_2_err =  pitch - setpoint;//motors 1 and 3

    // float roll_1_err =  //motors 0 and 1
    // float roll_2_err =  //motors 2 and 3


 return 0;
}