/*
FileName: Pitching_angle_PID_Controller.ino
Last updated: 17 May 2022
Author: Adithya Praveen
Description: This program is used to store the pitch angle in the SD Card and produce PID output corresponding the pitch angle and the reference pitch angle. File name in which angle is saved is: pitchAng.txt. Y angle from MPU6050 is the pitch angle
Revision Notes:
1. Placed calling of pid function outside the if loop
of writing of angle to SD Card
2. Tried to arrange the code according to Barr C Standards
3. In setup block, a starting sentence is added to the file in SD Card to distinguish it from old data previously stored.
4. Convert PID output to int value
5. Kp changed from 0.5 to 1.2
6. Kd changed from 0.3 to 0
7. Servo min and max angle changed from +/-20 to +/-40
*/

#include <MPU6050_light.h>// load mpu6050 library
#include <AutoPID.h>//library for pid auto tune
#include <Wire.h>
#include <I2CScanner.h>
#include <Servo.h> // library for servo fns
#include <SD.h> //Load SD card library
#include<SPI.h> //Load SPI Library

//Reference Angle for the Rocket
  double setpoint=0;

//Variables related with MPU6050
  double pitch;//for saving pitch angle
  double xangle, yangle, zangle;// for storing x, y,z angles
  unsigned long myTime; // To store sample time
  
  // Defining MPU6050 Function as mpu
    MPU6050 mpu(Wire);


//Variables and Objects related with SD Card
  int chipSelect = 10; //chipSelect pin for the SD card Reader
  File angleFile; //Data object you will write your sesnor data to
  
//Variables related with Servo
  Servo Xservo;// pitch servo
  int servoAngleDefault = 85 ; //Default servoAngle i.e. 0th posn of TVC
  int servopin_X=3;//servo X pin
  int servoCommandAngle;



// Variables and etc. related with PID controller
  double Output_max=40;//30 // servo maximum rotation from 90
  double Output_min=-40;//30 //servo minimum rotation from 90
  double Kp=1.2;//0.2 //proportional gain
  double Ki=0.01;//0.03 //integral gain
  double Kd=0;//0.1 // derivative gain
  double pidXout; // pitch output from PID

  // Defining PID Controller Function as pitchPID
    AutoPID pitchPID(&pitch,&setpoint,&pidXout,Output_min,Output_max, Kp, Ki, Kd);


void setup()
{
  // SD card intialization
    // Open serial communications and wait for port to open:
    Serial.begin(9600);
    while (!Serial)
    {
      ; // wait for serial port to connect. Needed for native USB port only
    }

    Serial.print("Initializing SD card...");

    if (!SD.begin(10))
    {
      Serial.println("initialization failed!");
      while (1);
    }
    Serial.println("initialization done.");

  // Intialization of MPU6050
    Wire.begin();
    Serial.begin(9600);
    Serial.println("Initialize MPU");
    byte status = mpu.begin();
    Serial.println(status);
    while(status!=0)
    { } // stop everything if could not connect to MPU6050
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    // mpu.upsideDownMounting = true; //uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); //To calculate gyro and accelerometer offset
    Serial.println("Done!\n");

  //Initialize the SD card reader
    SD.begin(10); 

  //Initialize the servo at pin D3
    Xservo.attach(servopin_X);
    Xservo.write(0);
    Serial.println("Servo Angle =0"); // To display on screen
    delay(2000);

  //Print the first line in the SD Card to distinguish the new readings from old
    angleFile = SD.open("pitchAng.txt",FILE_WRITE); 
    if (angleFile)
    {
      angleFile.print("Reading Start Here: ");
      angleFile.close();
    }

  //Flag signal to alert when to ignite 
    //Servo rotation as flag signal for igniting
    Xservo.write(125);
    Serial.println("Servo Angle =125"); // To display on screen
    delay(10000); //10s delay
    Xservo.write(servoAngleDefault);// Flag Servo signal at 90 deg (physical)
    Serial.println("Servo Angle = 85"); // To display on screen
    delay(2000);
  pitchPID.setTimeStep(40);//set PID update interval to 40ms

}

void loop()
{
  Serial.println("Loop started"); // To display on screen that the loop has started
  
  mpu.update();

  // Measurement File saved in SD Card as pitchAng.txt
  
  myTime = millis(); // To store the sammpling time
  angleFile = SD.open("pitchAng.txt",FILE_WRITE); 
  if (angleFile)
  {
    pitch= mpu.getAngleY();
    angleFile.print(myTime);
    angleFile.print(",");
    angleFile.println(pitch); //write angle data to card 
    angleFile.close();
    
    Serial.println("pitch"); // To display pitch angle on screen
    Serial.println(pitch);
    delay(100);
  }

  
  //PID Controller
    pitchPID.run();
    pidXout=servoAngleDefault+pidXout;//servoAngleDefault (i.e. offset of servo = 85 deg)
    servoCommandAngle = (int) pidXout;
    Xservo.write(servoCommandAngle); //Giving the command angle to the servo
    
    //To display PID output on screen
    Serial.println("servoCommandAngle");
    Serial.println(servoCommandAngle);

    delay(100);
  
  
}
