////////////////////////
// Forward Kinematics //
////////////////////////

#include <Servo.h>
#include <math.h>

// Arm Servo pins
#define Joint1Pin 2
#define Joint2Pin 3
#define Joint3Pin 4
#define GripperPin 9

// Servo Objects
Servo Joint1;
Servo Joint2;
Servo Joint3;
Servo Gripper;

// X, Y, Z Values
float x = 90;
float y = 90;
float z = 90;

// Link lengths (mm)
float l1 = 100;
float l2 = 170;

// Declare thetas
float theta_1;
float theta_2;
float theta_3;

// Starting Joint Angles
int Joint1Angle = 124; // Change 5 sets of angles
int Joint2Angle = 83; // Change 5 sets of angles
int Joint3Angle = 78; // Change 5 sets of angles
int GripperOpen = 150; // Open gripper; Need to tune value
int GripperClose = 60; // Close gripper; Need to tune value

// Joint Angle Offsets
int Joint1Offset = 10; // Your value may be different
int Joint2Offset = 15; // Your value may be different
int Joint3Offset = -14; // Your value may be different

void setup()
{
 Serial.begin(9600);
 Joint1.attach(Joint1Pin);
 Joint2.attach(Joint2Pin);
 Joint3.attach(Joint3Pin);
 Gripper.attach(GripperPin);
 
 // Write initial values
 Joint1.write(Joint1Angle+Joint1Offset); 
 Joint2.write(Joint2Angle+Joint2Offset); 
 Joint3.write(Joint3Angle+Joint3Offset); 
 Gripper.write(GripperOpen); // Open gripper

 // Calculate new values using inverse kinematics.

 // Calculate theta 3
 float raw_value = (pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2);
 theta_3 = acos(raw_value);

 // Calculate theta 2
 float thi = atan2(z, (pow(x, 2) + pow(y, 2)));
 float c = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
 float raw_beta = (pow(l1, 2) + pow(c, 2) - pow(l2, 2)) / (2 * l1 * l2);
 float beta = acos(raw_beta);
 theta_2 = thi + beta;

 // Calculate theta 1
 float raw_theta_1 = (y) / sqrt((pow(x, 2) + pow(y, 2)));
 theta_1 = acos(raw_theta_1);

 Serial.println("Test");
 Serial.println((theta_1 * 180) / PI);
 Serial.println((theta_2 * 180) / PI);
 Serial.println((theta_3 * 180) / PI);

 delay(5);
}

void loop()
{
 Joint1.write(theta_1+Joint1Offset); 
 Joint2.write(theta_2+Joint2Offset); 
 Joint3.write(theta_3+Joint3Offset); 
 delay(10);
}