//Special thanks to the guy of https://www.thepoorengineer.com/en/quaternion/ 
//ALL ANGLES IN RADIANS

#include <Wire.h>

#include <BasicLinearAlgebra.h> //will be ported to Eigen in the future
using namespace BLA;

//CALIBRATION
double gyro_cal[]={0,0,0}; //Fill with your calibration values and set calflag to false
double acc_cal[]={0,0,0};
bool calflag=true;

long accelX, accelY, accelZ;
double acc[]={0,0,0}, acc_total_vector;

long gyroY,gyroX,gyroZ;
double gyro[]={0,0,0};

double orientation[]={0,0,0}; //ROLL, PITCH, YAW GLOBAL
double orientation_local[]={0,0,0}; //ROLL, PITCH, YAW LOCAL

//QUATERNIONS
double c11, c12, c13, c21, c22, c23, c31, c32, c33;

BLA::Matrix<4,4> Sw = {0, 0, 0, 0,
                      0, 0, 0, 0,
                      0, 0, 0, 0,
                      0, 0, 0, 0};

BLA::Matrix<4,3> Sq = {0, 0, 0, 
                      0, 0, 0, 
                      0, 0, 0, 
                      0, 0, 0};
                      
BLA::Matrix<3,1> w = {0,
                      0,
                      0}; 
BLA::Matrix<3,1> orientation_x = {0,
                      0,
                      0}; 
                      
BLA::Matrix<3,1> orientation_y = {0,
                      0,
                      0};
                      
BLA::Matrix<3,1> orientation_z = {0,
                      0,
                      0};

BLA::Matrix<3,1> unit_x = {1,
                      0,
                      0}; 

BLA::Matrix<3,1> unit_y = {0,
                      1,
                      0};

BLA::Matrix<3,1> unit_z = {0,
                      0,
                      1};
                                                               
BLA::Matrix<4,1> q = {1,
                      0,
                      0,
                      0};
                       
BLA::Matrix<4,1> q_local = {1,
                      0,
                      0,
                      0}; 

BLA::Matrix<3,3> C = {0, 0, 0,
                      0, 0, 0,
                      0, 0, 0};

double dot;

//TIMES
double T=0.01,dt; //T is the sample time of your program, change it accordingly
unsigned long currentTime,previousTime;
unsigned long timer_run;

void setup() {
  Serial.begin(250000);
  Wire.begin();
  setupMPU();
Serial.print("Gyro calibration");
  recordAccelRegisters();
  recordGyroRegisters();

if(calflag==true){
  
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++){                  //Run this code 1000 times
    recordGyroRegisters();                                                 
    gyro_cal[0] += gyroX;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_cal[1] += gyroY;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_cal[2] += gyroZ;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  
  gyro_cal[0] /= (1000);                                                  //65.5 RAW TO DEG
  gyro_cal[1] /= (1000);                                                  //Divide the gyro_y_cal variable by 1000 to get the avarage offset
  gyro_cal[2] /= (1000);   }
  

}


void loop() {
if(micros()>=timer_run+T*1000000UL){ // T is going to be replaced by dt in the future
  dt=(micros()-previousTime)/1000000UL;
  previousTime=micros();
  timer_run=micros();


  
  recordAccelRegisters();
  recordGyroRegisters();
  //printData();
  calculate_orientation();
  printData2();


  

}}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0b00011000); //Setting the gyro to full scale +/- 2000deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00011000); //Setting the accel to +/- 16g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData(){
  acc[1] = (accelX / 2048.0);
  acc[0] = (accelY / 2048.0); 
  acc[2] = (-accelZ / 2048.0)-0.07;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ


  
  processGyroData();
}

void processGyroData() {
  gyro[1] = ((gyroX-gyro_cal[0]) / (16.375*57.295));
  gyro[0]= (gyroY-gyro_cal[1]) / (16.375*57.295); 
  gyro[2]= -(gyroZ-gyro_cal[2]) / (16.375*57.295);
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(gyro[0]*57.3,6);
  Serial.print(" Y=");
  Serial.print(gyro[1]*57.3,6);
  Serial.print(" Z=");
  Serial.print(gyro[2]*57.3,6);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(acc[0],6);
  Serial.print(" Y=");
  Serial.print(acc[1],6);
  Serial.print(" Z=");
  Serial.println(acc[2],6);
  
}

void printData2() {
//  Serial.print(orientation[0]*57.3,6);
//  Serial.print(" , ");
//  Serial.print(orientation[1]*57.3,6);
//  Serial.print(" , ");
//  Serial.println(orientation[2]*57.3,6);


  
  Serial.print(orientation[0]*57.3);
  Serial.print(" , ");
  Serial.print(orientation_local[1]*57.3);
  Serial.print(" , ");
  Serial.println(orientation_local[2]*57.3);
//
//  Serial.print(sqrt(sq(orientation_x(0,0))+sq(orientation_x(1,0))+sq(orientation_x(2,0))),6);
//  Serial.print(" , ");
//  Serial << "orientation_matrix: " << orientation_x << '\n';
//  
//  Serial.print(" , ");
//Serial.println(dt,6);

}


//ORIENTATION 

void calculate_orientation(){

w<< gyro[0], //LOCAL ROLL RATE
    gyro[1], //LOCAL PITCH RATE
    gyro[2]; //LOCAL YAW RATE
    
Sq<< -q(1,0),-q(2,0),-q(3,0), 
     q(0,0),-q(3,0),q(2,0),
     q(3,0),q(0,0),-q(1,0),
     -q(2,0),q(1,0),q(0,0);

q_local=Sq*w; //BLA DOESN'T WANT TO DO ALL THE OPERATIONS IN ONE LINE
q_local(0,0)=0.5*T*q_local(0,0); //Euler integration, will be changed to trapezoidal in the future
q_local(1,0)=0.5*T*q_local(1,0);
q_local(2,0)=0.5*T*q_local(2,0);
q_local(3,0)=0.5*T*q_local(3,0);
q=q_local+q; //Euler integration, will be changed to trapezoidal in the future

double quat_mod=sqrt((sq(q(0,0))+sq(q(1,0))+sq(q(2,0))+sq(q(3,0)))); //NORMALIZATION OF QUATERNION

q(0,0)=q(0,0)/quat_mod;
q(1,0)=q(1,0)/quat_mod;
q(2,0)=q(2,0)/quat_mod;
q(3,0)=q(3,0)/quat_mod;

c12=2*(q(1,0)*q(2,0)-q(3,0)*q(0,0));
c13=2*(q(1,0)*q(3,0)+q(2,0)*q(0,0));
c21=2*(q(1,0)*q(2,0)+q(3,0)*q(0,0));
c23=2*(q(2,0)*q(3,0)-q(1,0)*q(0,0));
c31=2*(q(1,0)*q(3,0)-q(2,0)*q(0,0));
c32=2*(q(2,0)*q(3,0)+q(1,0)*q(0,0));



C<< (1-2*(sq(q(2,0))+sq(q(3,0)))), (c12), (c13), //CREATES THE ROTATION MATRIX

    (c21), (1-2*(sq(q(1,0))+sq(q(3,0)))), (c23),

    (c31), (c32), (1-2*(sq(q(1,0))+sq(q(2,0))));


orientation_x=C*unit_x; //Computes the new base
orientation_y=C*unit_y;
orientation_z=C*unit_z;


//COMPUTES GLOBAL ORIENTATION //STILL EULER ANGLES, READINGS ARE NOT GOOD FOR HIGH PITCH ANGLES
orientation[0]=-atan2(C(1,2),C(2,2));  //ROLL
orientation[1]=-asin((orientation_x(2,0))/(1)); //GLOBAL PITCH
orientation[2]=asin((orientation_x(1,0))/(sqrt(sq(orientation_x(1,0))+sq(orientation_x(0,0))))); //GLOBAL YAW 




//LOCAL ANGLES, THIS ARE THE ONES YOU SHOULD FEED TO THE CONTROL SYSTEM
orientation_local[0]=-atan2(C(1,2),C(2,2)); //ROLL
orientation_local[1]=asin(C(0,2)); //PITCH
orientation_local[2]=-atan2(C(0,1),C(0,0)); //YAW
}
