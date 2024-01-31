////////////////////////
// Forward Kinematics //
////////////////////////

#include <Servo.h>

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

// Starting Joint Angles
int Joint1Angle = 60; // Change 5 sets of angles
int Joint2Angle = 60; // Change 5 sets of angles
int Joint3Angle = 60; // Change 5 sets of angles
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
 
 Joint1.write(Joint1Angle+Joint1Offset); 
 Joint2.write(Joint2Angle+Joint2Offset); 
 Joint3.write(Joint3Angle+Joint3Offset); 
 Gripper.write(GripperOpen); // Open gripper
 
 delay(5000); // 5 seconds before loop function
}

void loop()
{
 Joint1.write(Joint1Angle+Joint1Offset); 
 Joint2.write(Joint2Angle+Joint2Offset); 
 Joint3.write(Joint3Angle+Joint3Offset); 
 Gripper.write(GripperOpen); // Open gripper
 
 delay(10);
}