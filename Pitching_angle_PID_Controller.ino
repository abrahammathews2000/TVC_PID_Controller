/*
*
Y angle is the pitch angle
File: Pitch_v1
Last updated: 16 May 2022
Author: Adithya Praveen
Last updated by: Abraham Mathews (13:04 pm, 16 May
2022)
Description: This program is used to store the
pitch angle in the SD Card and produce PID output
corresponding the
pitch angle and the reference pitch angle. File
name in which angle is saved is: pitchAng.txt

Revision Notes:
Placed calling of pid function outside the if loop
of writing of angle to SD Card
*/

#include <MPU6050_light.h>// load mpu6050 library
#include <AutoPID.h>//library for pid auto tune
#include <Wire.h>
#include <I2CScanner.h>
#include <Servo.h> // library for servo fns
#include <SD.h> //Load SD card library
#include<SPI.h> //Load SPI Library

MPU6050 mpu(Wire);
int val_x;//val_x:offset for pitch;
double pitch;//for saving pitch angle

double xangle, yangle, zangle;// for storing x, y,z angles

int chipSelect = 10; //chipSelect pin for the SD card Reader
File angleFile; //Data object you will write your sesnor data to
unsigned long myTime;

double Output_max=20;//30 // servo maximum rotation from 90
double Output_min=-20;//30 //servo minimum rotation from 90
double Kp=0.5;//0.2 //proportional gain
double Ki=0.01;//0.03 //integral gain
double Kd=0.3;//0.1 // derivative gain
double setpoint=0; //reference angle
double pidXout; // pitch output from PID
AutoPID pitchPID(&pitch,&setpoint,&pidXout,Output_min,Output_max, Kp, Ki, Kd);// autopid_pitch

Servo Xservo;// pitch servo
int servoAngleDefault = 85 ; //Default servoAngle i.e. 0th posn of TVC
int servopin_X=3;//servo X pin

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

// intialization mpu
Wire.begin();
Serial.begin(9600);
Serial.println("Initialize MPU");
byte status = mpu.begin();
Serial.println(status);
while(status!=0){ } // stop everything if could not connect to MPU6050
Serial.println(F("Calculating offsets, do not move MPU6050"));
// mpu.upsideDownMounting = true; //uncomment this line if the MPU6050 is mounted upside-down
mpu.calcOffsets(); // gyro and accelero
Serial.println("Done!\n");

SD.begin(10); //Initialize the SD card reader

Xservo.attach(servopin_X);
Xservo.write(0);
Serial.println("Servo Angle =0"); // To display on screen

delay(2000);

//Servo rotation as flag signal for igniting
Xservo.write(125);
Serial.println("Servo Angle =125"); // To display on screen
delay(10000); //10s delay
Xservo.write(85);// Flag Servo signal at 90 deg (physical) =85 deg(electrical)
Serial.println("Servo Angle =85"); // To display on screen
delay(2000);
pitchPID.setTimeStep(40);//set PID update interval to 40ms

}

void loop()
{Serial.println("Loop started"); // To display on screen
mpu.update();

myTime = millis();

//xangle= mpu.getAngleX(); // Read xangle from MPU6050
//yangle = mpu.getAngleY(); // Yangle = Pitch angle
//zangle= mpu.getAngleZ(); //zangle
angleFile = SD.open("pitchAng.txt",FILE_WRITE); // File saved as SAMPLE.txt
if (angleFile)
{
/* To print on screen of computer
* Serial.print("The X_Angle :"); //Print
Your results
Serial.print(xangle);
Serial.print("The Y_Angle : ");
Serial.print(yangle);
Serial.print("The Z_Angle :");
Serial.print(zangle);
Serial.println(""); */
pitch= mpu.getAngleY();
angleFile.print(myTime);
angleFile.print(",");
angleFile.
println(pitch); //write angle data to card 
angleFile.close();
Serial.println("pitch");
Serial.println(pitch);
delay(100);
}

pitchPID.run();

pidXout=servoAngleDefault+pidXout;//offset of pitch servo 85
Serial.println("pidXout");
Serial.println(pidXout);
Xservo.write(pidXout);
delay(100);
}
