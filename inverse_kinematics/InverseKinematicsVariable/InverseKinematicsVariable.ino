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

// Control Pins
int Joint1ControlPin = A1;
int Joint2ControlPin = A2;
int Joint3ControlPin = A3;

// Control Values
int Joint1Control = 512; // middle value between 0 and 1024
int Joint2Control = 512; // middle value between 0 and 1024
int Joint3Control = 512;

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
float l1 = 90;
float l2 = 170;

// Declare thetas
float theta_1;
float theta_2;
float theta_3;

// Starting Joint Angles
int Joint1Angle = 90; // Change 5 sets of angles
int Joint2Angle = 90; // Change 5 sets of angles
int Joint3Angle = 90; // Change 5 sets of angles
int GripperOpen = 150; // Open gripper; Need to tune value
int GripperClose = 60; // Close gripper; Need to tune value

// Joint Angle Offsets
int Joint1Offset = 10; // Your value may be different
int Joint2Offset = 15; // Your value may be different
int Joint3Offset = -14; // Your value may be different

void calculate_theta_1()
{
theta_1 = acos(x / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
}

void calculate_theta_2()
{
float phi = atan(-1*z / sqrt(pow(x, 2) + pow(y, 2))) * (180 / PI);
float beta_top = pow(l1, 2) + pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(l2, 2);
float beta_bottom = (2 * l1 * sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)));
float beta = acos((beta_top / beta_bottom)) * (180 / PI);
theta_2 = phi + beta;
}

void calculate_theta_3()
{
theta_3 = acos(((pow(x, 2) + pow(y, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2)) / (2 * l1 * l2))) * (180 / PI);
}

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
 Gripper.write(GripperOpen);
 delay(5);
}

void loop()
{
 // Read Potentiometer Values
 float read_x = analogRead(Joint1ControlPin);
 float read_y = analogRead(Joint2ControlPin);
 float read_z = analogRead(Joint3ControlPin);

 x = map(read_x,0,1023,0,240);
 y = map(read_y,0,1023,0,150);
 z = map(read_z,0,1023,0,-240);

 calculate_theta_1();
 calculate_theta_2();
 calculate_theta_3();
 
 Joint1.write(theta_1+Joint1Offset); 
 Joint2.write(theta_2+Joint2Offset); 
 Joint3.write(theta_3+Joint3Offset); 

 delay(10);
}